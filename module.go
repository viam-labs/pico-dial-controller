package picodialcontroller

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/bearsh/hid"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	generic "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

const (
	vendorID  uint16 = 0x2E8A
	productID uint16 = 0x000A

	defaultMoveMM    float64 = 1.0
	defaultDrainMs   int     = 20 // 50 Hz
)

var PicoDialController = resource.NewModel("viam", "pico-dial-controller", "pico-dial-controller")

func init() {
	resource.RegisterComponent(generic.API, PicoDialController,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newPicoDialControllerPicoDialController,
		},
	)
}

// DialMapping maps a physical encoder index to an arm axis.
type DialMapping struct {
	Dial int    `json:"dial"`
	Axis string `json:"axis"` // "x", "y", "z", "orientation", "rx", "ry", "rz"
}

// Config holds the module configuration.
type Config struct {
	// Serial is the USB serial number of the Pico to use.
	// Omit to connect to the first device found with the matching VID/PID.
	Serial string `json:"serial,omitempty"`

	ArmName string        `json:"arm_name"`
	Dials   []DialMapping `json:"dials"`

	DialMoveXMM           float64 `json:"dial_move_x_mm,omitempty"`
	DialMoveYMM           float64 `json:"dial_move_y_mm,omitempty"`
	DialMoveZMM           float64 `json:"dial_move_z_mm,omitempty"`
	DialMoveOrientationMM float64 `json:"dial_move_orientation_mm,omitempty"`

	DialMoveRXDeg float64 `json:"dial_move_rx_deg,omitempty"`
	DialMoveRYDeg float64 `json:"dial_move_ry_deg,omitempty"`
	DialMoveRZDeg float64 `json:"dial_move_rz_deg,omitempty"`

	// DrainIntervalMs controls how often accumulated dial input is flushed to
	// the arm. Detents that arrive between flushes are summed, so faster
	// turning produces proportionally larger moves per flush (natural
	// acceleration). Lower values are more responsive; higher values give
	// smoother motion. Default: 20ms (50 Hz).
	DrainIntervalMs int `json:"drain_interval_ms,omitempty"`

	// Acceleration: movement multiplier grows as more detents arrive per flush window.
	// multiplier = clamp((count / AccelThresholdCount)^AccelExponent, 1, AccelMaxMultiplier)
	// Single detents (fine control) stay at 1× until count exceeds the threshold.
	AccelThresholdCount float64 `json:"accel_threshold_count,omitempty"` // detents/window to start accelerating (default 2)
	AccelMaxMultiplier  float64 `json:"accel_max_multiplier,omitempty"`  // max multiplier at high speed (default 10)
	AccelExponent       float64 `json:"accel_exponent,omitempty"`        // curve shape: 1=linear, 2=quadratic (default 1.5)
}

func (cfg *Config) drainInterval() time.Duration {
	if cfg.DrainIntervalMs > 0 {
		return time.Duration(cfg.DrainIntervalMs) * time.Millisecond
	}
	return time.Duration(defaultDrainMs) * time.Millisecond
}

func (cfg *Config) accelThresholdCount() float64 {
	if cfg.AccelThresholdCount > 0 {
		return cfg.AccelThresholdCount
	}
	return 2.0
}

func (cfg *Config) accelMaxMultiplier() float64 {
	if cfg.AccelMaxMultiplier > 0 {
		return cfg.AccelMaxMultiplier
	}
	return 10.0
}

func (cfg *Config) accelExponent() float64 {
	if cfg.AccelExponent > 0 {
		return cfg.AccelExponent
	}
	return 1.5
}

func (cfg *Config) moveMM(axis string) float64 {
	switch axis {
	case "x":
		if cfg.DialMoveXMM > 0 {
			return cfg.DialMoveXMM
		}
	case "y":
		if cfg.DialMoveYMM > 0 {
			return cfg.DialMoveYMM
		}
	case "z":
		if cfg.DialMoveZMM > 0 {
			return cfg.DialMoveZMM
		}
	case "orientation":
		if cfg.DialMoveOrientationMM > 0 {
			return cfg.DialMoveOrientationMM
		}
	}
	return defaultMoveMM
}

func (cfg *Config) moveDeg(axis string) float64 {
	switch axis {
	case "rx":
		if cfg.DialMoveRXDeg > 0 {
			return cfg.DialMoveRXDeg
		}
	case "ry":
		if cfg.DialMoveRYDeg > 0 {
			return cfg.DialMoveRYDeg
		}
	case "rz":
		if cfg.DialMoveRZDeg > 0 {
			return cfg.DialMoveRZDeg
		}
	}
	return 1.0
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.ArmName == "" {
		return nil, nil, fmt.Errorf("%s: arm_name is required", path)
	}
	if len(cfg.Dials) == 0 {
		return nil, nil, fmt.Errorf("%s: dials must not be empty", path)
	}
	for i, d := range cfg.Dials {
		switch d.Axis {
		case "x", "y", "z", "orientation", "rx", "ry", "rz":
		default:
			return nil, nil, fmt.Errorf("%s: dials[%d] axis %q must be one of: x, y, z, orientation, rx, ry, rz", path, i, d.Axis)
		}
	}
	return []string{cfg.ArmName}, nil, nil
}

type picoDialControllerPicoDialController struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *Config
	cancelCtx  context.Context
	cancelFunc func()
	myArm      arm.Arm
	byDial     map[int]DialMapping

	pendingMu     sync.Mutex
	pendingMoves  map[string]float64 // axis -> accumulated delta (mm or deg)
	pendingCounts map[string]int     // axis -> detent count this window (for acceleration)
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

	myArm, err := arm.FromDependencies(deps, conf.ArmName)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("arm %q not found: %w", conf.ArmName, err)
	}

	byDial := make(map[int]DialMapping, len(conf.Dials))
	for _, d := range conf.Dials {
		byDial[d.Dial] = d
	}

	s := &picoDialControllerPicoDialController{
		name:         name,
		logger:       logger,
		cfg:          conf,
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
		myArm:        myArm,
		byDial:       byDial,
		pendingMoves:  make(map[string]float64),
		pendingCounts: make(map[string]int),
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
	go s.drainLoop()
	return s, nil
}

// readLoop reads HID detent events and accumulates them into pendingMoves.
// It does not call the arm directly — that is drainLoop's job.
func (s *picoDialControllerPicoDialController) readLoop(dev *hid.Device) {
	defer dev.Close()

	buf := make([]byte, 64)
	for {
		select {
		case <-s.cancelCtx.Done():
			return
		default:
		}

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
			continue
		}

		var delta float64
		switch mapping.Axis {
		case "rx", "ry", "rz":
			delta = s.cfg.moveDeg(mapping.Axis) * float64(direction)
		default:
			delta = s.cfg.moveMM(mapping.Axis) * float64(direction)
		}

		s.pendingMu.Lock()
		s.pendingMoves[mapping.Axis] += delta
		s.pendingCounts[mapping.Axis]++
		s.pendingMu.Unlock()
	}
}

// drainLoop ticks at drain_interval_ms and sends one arm command per tick
// containing all accumulated movement since the last tick. When the dial is
// still, pendingMoves is empty and no arm command is issued.
func (s *picoDialControllerPicoDialController) drainLoop() {
	ticker := time.NewTicker(s.cfg.drainInterval())
	defer ticker.Stop()

	for {
		select {
		case <-s.cancelCtx.Done():
			return
		case <-ticker.C:
			s.pendingMu.Lock()
			if len(s.pendingMoves) == 0 {
				s.pendingMu.Unlock()
				continue
			}
			pending := s.pendingMoves
			counts := s.pendingCounts
			s.pendingMoves = make(map[string]float64)
			s.pendingCounts = make(map[string]int)
			s.pendingMu.Unlock()

			if err := s.flushMoves(pending, counts); err != nil {
				s.logger.Warnw("arm move failed", "error", err)
			}
		}
	}
}

// accelMultiplier returns the movement multiplier for a given detent count
// within one drain window. Velocity is counts/drain_interval_seconds, giving a
// per-batch multiplier that goes to zero as soon as no detents arrive.
func (s *picoDialControllerPicoDialController) accelMultiplier(count int) float64 {
	threshold := s.cfg.accelThresholdCount()
	if float64(count) <= threshold {
		return 1.0
	}
	multiplier := math.Pow(float64(count)/threshold, s.cfg.accelExponent())
	return math.Min(multiplier, s.cfg.accelMaxMultiplier())
}

// flushMoves applies all accumulated deltas in a single EndPosition /
// MoveToPosition round-trip. Translation axes are applied to the position
// vector; rotation axes are composed onto the orientation quaternion.
// An acceleration multiplier is derived from the detent count per axis.
func (s *picoDialControllerPicoDialController) flushMoves(pending map[string]float64, counts map[string]int) error {
	currentPose, err := s.myArm.EndPosition(s.cancelCtx, nil)
	if err != nil {
		return fmt.Errorf("failed to get arm position: %w", err)
	}

	pt := currentPose.Point()
	ori := currentPose.Orientation()

	// Apply translation deltas first.
	for axis, delta := range pending {
		delta *= s.accelMultiplier(counts[axis])
		switch axis {
		case "x":
			pt.X += delta
		case "y":
			pt.Y += delta
		case "z":
			pt.Z += delta
		case "orientation":
			ov := ori.OrientationVectorRadians()
			pt = r3.Vector{
				X: pt.X + ov.OX*delta,
				Y: pt.Y + ov.OY*delta,
				Z: pt.Z + ov.OZ*delta,
			}
		}
	}

	// Apply rotation deltas, updating orientation incrementally.
	for _, axis := range []string{"rx", "ry", "rz"} {
		delta, ok := pending[axis]
		if !ok {
			continue
		}
		delta *= s.accelMultiplier(counts[axis])
		newPose, err := rotatePose(spatialmath.NewPose(pt, ori), axis, delta)
		if err != nil {
			return err
		}
		pt = newPose.Point()
		ori = newPose.Orientation()
	}

	return s.myArm.MoveToPosition(s.cancelCtx, spatialmath.NewPose(pt, ori), nil)
}

// rotatePose applies a rotation of deg degrees around the given world axis (rx/ry/rz)
// to the pose's orientation by left-multiplying the rotation quaternion.
func rotatePose(pose spatialmath.Pose, axis string, deg float64) (spatialmath.Pose, error) {
	theta := deg * math.Pi / 180.0
	half := theta / 2.0
	cosHalf, sinHalf := math.Cos(half), math.Sin(half)

	var rotQ quat.Number
	switch axis {
	case "rx":
		rotQ = quat.Number{Real: cosHalf, Imag: sinHalf}
	case "ry":
		rotQ = quat.Number{Real: cosHalf, Jmag: sinHalf}
	case "rz":
		rotQ = quat.Number{Real: cosHalf, Kmag: sinHalf}
	default:
		return nil, fmt.Errorf("unknown rotation axis %q", axis)
	}

	currentQ := pose.Orientation().Quaternion()
	newQ := quat.Mul(rotQ, quat.Number{
		Real: currentQ.Real,
		Imag: currentQ.Imag,
		Jmag: currentQ.Jmag,
		Kmag: currentQ.Kmag,
	})

	// Normalize
	mag := quat.Abs(newQ)
	if mag > 1e-10 {
		newQ = quat.Scale(1.0/mag, newQ)
	}

	// Convert quaternion to R4AA (axis-angle)
	w := math.Min(1.0, math.Max(-1.0, newQ.Real))
	theta2 := 2.0 * math.Acos(w)
	s := math.Sin(theta2 / 2.0)

	var ori spatialmath.Orientation
	if s < 1e-10 {
		ori = &spatialmath.R4AA{Theta: 0, RX: 1}
	} else {
		ori = &spatialmath.R4AA{
			Theta: theta2,
			RX:    newQ.Imag / s,
			RY:    newQ.Jmag / s,
			RZ:    newQ.Kmag / s,
		}
	}

	return spatialmath.NewPose(pose.Point(), ori), nil
}

func (s *picoDialControllerPicoDialController) Name() resource.Name {
	return s.name
}

func (s *picoDialControllerPicoDialController) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{
		"dials_configured": len(s.cfg.Dials),
		"arm":              s.cfg.ArmName,
	}, nil
}

func (s *picoDialControllerPicoDialController) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{
		"dials_configured": len(s.cfg.Dials),
		"arm":              s.cfg.ArmName,
	}, nil
}

func (s *picoDialControllerPicoDialController) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
