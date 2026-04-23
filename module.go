package picodialcontroller

import (
	"context"
	"fmt"

	"github.com/bearsh/hid"
	generic "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

const (
	vendorID  uint16 = 0x2E8A
	productID uint16 = 0x000A
)

var PicoDialController = resource.NewModel("viam", "pico-dial-controller", "pico-dial-controller")

func init() {
	resource.RegisterComponent(generic.API, PicoDialController,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newPicoDialControllerPicoDialController,
		},
	)
}

// DialMapping maps a physical encoder index to a Viam component and command.
type DialMapping struct {
	Dial      int    `json:"dial"`
	Component string `json:"component"`
	Command   string `json:"command"`
}

// Config holds the module configuration.
type Config struct {
	// Serial is the USB serial number of the Pico to use.
	// Omit to connect to the first device found with the matching VID/PID.
	Serial string        `json:"serial,omitempty"`
	Dials  []DialMapping `json:"dials"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if len(cfg.Dials) == 0 {
		return nil, nil, fmt.Errorf("%s: dials must not be empty", path)
	}
	seen := map[string]bool{}
	var deps []string
	for i, d := range cfg.Dials {
		if d.Component == "" {
			return nil, nil, fmt.Errorf("%s: dials[%d] missing component", path, i)
		}
		if d.Command == "" {
			return nil, nil, fmt.Errorf("%s: dials[%d] missing command", path, i)
		}
		if !seen[d.Component] {
			deps = append(deps, d.Component)
			seen[d.Component] = true
		}
	}
	return deps, nil, nil
}

type picoDialControllerPicoDialController struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *Config
	cancelCtx  context.Context
	cancelFunc func()
	deps       resource.Dependencies
	byDial     map[int]DialMapping
}

func newPicoDialControllerPicoDialController(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewPicoDialController(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewPicoDialController(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	byDial := make(map[int]DialMapping, len(conf.Dials))
	for _, d := range conf.Dials {
		byDial[d.Dial] = d
	}

	s := &picoDialControllerPicoDialController{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		deps:       deps,
		byDial:     byDial,
	}

	devs := hid.Enumerate(vendorID, productID)
	if len(devs) == 0 {
		cancelFunc()
		return nil, fmt.Errorf("no HID device found (VID=0x%04X PID=0x%04X)", vendorID, productID)
	}
	var devInfo *hid.DeviceInfo
	for i := range devs {
		if conf.Serial == "" || devs[i].Serial == conf.Serial {
			devInfo = &devs[i]
			break
		}
	}
	if devInfo == nil {
		cancelFunc()
		return nil, fmt.Errorf("no HID device found with serial %q", conf.Serial)
	}
	dev, err := devInfo.Open()
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to open HID device: %w", err)
	}

	go s.readLoop(dev)
	return s, nil
}

func (s *picoDialControllerPicoDialController) readLoop(dev *hid.Device) {
	defer dev.Close()

	buf := make([]byte, 64)
	for {
		select {
		case <-s.cancelCtx.Done():
			return
		default:
		}

		// 100ms timeout so we can check cancelCtx without blocking indefinitely
		n, err := dev.ReadTimeout(buf, 100)
		if err != nil {
			s.logger.Errorw("HID read error, stopping", "error", err)
			return
		}
		if n < 2 {
			continue
		}

		dialIndex := int(buf[0])
		direction := int(int8(buf[1])) // +1 or -1

		mapping, ok := s.byDial[dialIndex]
		if !ok {
			continue // encoder not mapped in config
		}

		compName := resource.NewName(generic.API, mapping.Component)
		res, ok := s.deps[compName]
		if !ok {
			s.logger.Errorw("component not found in dependencies", "component", mapping.Component)
			continue
		}

		if _, err := res.DoCommand(s.cancelCtx, map[string]interface{}{
			"command":   mapping.Command,
			"direction": direction,
		}); err != nil {
			s.logger.Warnw("DoCommand failed",
				"component", mapping.Component,
				"command", mapping.Command,
				"error", err,
			)
		}
	}
}

func (s *picoDialControllerPicoDialController) Name() resource.Name {
	return s.name
}

func (s *picoDialControllerPicoDialController) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{
		"dials_configured": len(s.cfg.Dials),
	}, nil
}

func (s *picoDialControllerPicoDialController) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{
		"dials_configured": len(s.cfg.Dials),
	}, nil
}

func (s *picoDialControllerPicoDialController) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
