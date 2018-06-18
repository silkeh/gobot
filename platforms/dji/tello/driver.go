package tello

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"net"
	"strconv"
	"sync"
	"time"

	"gobot.io/x/gobot"
)

const (
	// BounceEvent event
	BounceEvent = "bounce"

	// ConnectedEvent event
	ConnectedEvent = "connected"

	// FlightDataEvent event
	FlightDataEvent = "flightdata"

	// TakeoffEvent event
	TakeoffEvent = "takeoff"

	// LandingEvent event
	LandingEvent = "landing"

	// PalmLandingEvent event
	PalmLandingEvent = "palm-landing"

	// FlipEvent event
	FlipEvent = "flip"

	// TimeEvent event
	TimeEvent = "time"

	// LogEvent event
	LogEvent = "log"

	// LogDataEvent event
	LogDataEvent = "logdata"

	// WifiDataEvent event
	WifiDataEvent = "wifidata"

	// LightStrengthEvent event
	LightStrengthEvent = "lightstrength"

	// SetExposureEvent event
	SetExposureEvent = "setexposure"

	// VideoFrameEvent event
	VideoFrameEvent = "videoframe"

	// SetVideoEncoderRateEvent event
	SetVideoEncoderRateEvent = "setvideoencoder"
)

// the 16-bit messages and commands stored in bytes 6 & 5 of the packet
const (
	messageStart     = 0x00cc // 204
	wifiMessage      = 0x001a // 26
	videoRateQuery   = 0x0028 // 40
	lightMessage     = 0x0035 // 53
	flightMessage    = 0x0056 // 86
	jpegStartMessage = 0x0062 // 98
	jpegMessage      = 0x0063 // 99
	jpegEndMessage   = 0x0064 // 100
	logMessage       = 0x1050 // 4176
	logDataMessage   = 0x1051 // 4177
	logConfigMessage = 0x1052 // 4178
	maxHeightMessage = 0x1056 // 4182
	attAngleMessage  = 0x1059 // 4185

	videoEncoderRateCommand = 0x0020 // 32
	videoDynRateCommand     = 0x0021 // 33
	videoStartCommand       = 0x0025 // 37
	videoRecordCommand      = 0x0032 // 50
	exposureCommand         = 0x0034 // 52
	timeCommand             = 0x0046 // 70
	stickCommand            = 0x0050 // 80
	takeoffCommand          = 0x0054 // 84
	landCommand             = 0x0055 // 85
	flipCommand             = 0x005c // 92
	throwtakeoffCommand     = 0x005d // 93
	palmLandCommand         = 0x005e // 94
	bounceCommand           = 0x1053 // 4179
	maxHeightCommand        = 0x0058 // 88
	attAngleCommand         = 0x1058 // 4184
	eisCommand              = 0x0024 // 36
	pictureCommand          = 0x0030 // 48
	picVidModeCommand       = 0x0031 // 49
	jpegQualityCommand      = 0x0037 // 55
)

// The 8-bit message types
const (
	queryType  = 0x48
	setType    = 0x68
	ackType    = 0x50
	frameType  = 0x60
	configType = 0x88
	flipType   = 0x70
)

// FlipType is used for the various flips supported by the Tello.
type FlipType int

const (
	// FlipFront flips forward.
	FlipFront FlipType = 0

	// FlipLeft flips left.
	FlipLeft FlipType = 1

	// FlipBack flips backwards.
	FlipBack FlipType = 2

	// FlipRight flips to the right.
	FlipRight FlipType = 3

	// FlipForwardLeft flips forwards and to the left.
	FlipForwardLeft FlipType = 4

	// FlipBackLeft flips backwards and to the left.
	FlipBackLeft FlipType = 5

	// FlipBackRight flips backwards and to the right.
	FlipBackRight FlipType = 6

	// FlipForwardRight flips forewards and to the right.
	FlipForwardRight FlipType = 7
)

// VideoBitRate is used to set the bit rate for the streaming video returned by the Tello.
type VideoBitRate int

const (
	// VideoBitRateAuto sets the bitrate for streaming video to auto-adjust.
	VideoBitRateAuto VideoBitRate = 0

	// VideoBitRate1M sets the bitrate for streaming video to 1 Mb/s.
	VideoBitRate1M VideoBitRate = 1

	// VideoBitRate15M sets the bitrate for streaming video to 1.5 Mb/s
	VideoBitRate15M VideoBitRate = 2

	// VideoBitRate2M sets the bitrate for streaming video to 2 Mb/s.
	VideoBitRate2M VideoBitRate = 3

	// VideoBitRate3M sets the bitrate for streaming video to 3 Mb/s.
	VideoBitRate3M VideoBitRate = 4

	// VideoBitRate4M sets the bitrate for streaming video to 4 Mb/s.
	VideoBitRate4M VideoBitRate = 5
)

// ImageMode is used to switch between Photo and Video mode
type ImageMode int

const (
	// PictureMode sets the drone to picture mode
	PictureMode ImageMode = 0

	// VideoMode sets the drone to video mode
	VideoMode ImageMode = 1
)

// WifiData packet returned by the Tello
type WifiData struct {
	Disturb  int8
	Strength int8
}

// Driver represents the DJI Tello drone
type Driver struct {
	name           string
	reqAddr        string
	cmdConn        io.ReadWriteCloser // UDP connection to send/receive drone commands
	videoConn      *net.UDPConn       // UDP connection for drone video
	respPort       string
	videoPort      string
	cmdMutex       sync.Mutex
	seq            uint16
	rx, ry, lx, ly float32
	throttle       int
	bouncing       bool
	gobot.Eventer
}

// NewDriver creates a driver for the Tello drone. Pass in the UDP port to use for the responses
// from the drone.
func NewDriver(port string) *Driver {
	d := &Driver{name: gobot.DefaultName("Tello"),
		reqAddr:   "192.168.10.1:8889",
		respPort:  port,
		videoPort: "11111",
		Eventer:   gobot.NewEventer(),
	}

	d.AddEvent(ConnectedEvent)
	d.AddEvent(FlightDataEvent)
	d.AddEvent(TakeoffEvent)
	d.AddEvent(LandingEvent)
	d.AddEvent(PalmLandingEvent)
	d.AddEvent(BounceEvent)
	d.AddEvent(FlipEvent)
	d.AddEvent(TimeEvent)
	d.AddEvent(LogEvent)
	d.AddEvent(WifiDataEvent)
	d.AddEvent(LightStrengthEvent)
	d.AddEvent(SetExposureEvent)
	d.AddEvent(VideoFrameEvent)
	d.AddEvent(SetVideoEncoderRateEvent)

	return d
}

// Name returns the name of the device.
func (d *Driver) Name() string { return d.name }

// SetName sets the name of the device.
func (d *Driver) SetName(n string) { d.name = n }

// Connection returns the Connection of the device.
func (d *Driver) Connection() gobot.Connection { return nil }

// Start starts the driver.
func (d *Driver) Start() error {
	reqAddr, err := net.ResolveUDPAddr("udp", d.reqAddr)
	if err != nil {
		fmt.Println(err)
		return err
	}
	respPort, err := net.ResolveUDPAddr("udp", ":"+d.respPort)
	if err != nil {
		fmt.Println(err)
		return err
	}
	cmdConn, err := net.DialUDP("udp", respPort, reqAddr)
	if err != nil {
		fmt.Println(err)
		return err
	}
	d.cmdConn = cmdConn

	// handle responses
	go func() {
		d.On(d.Event(ConnectedEvent), func(interface{}) {
			d.SendDateTime()
			d.processVideo()
		})

		for {
			err := d.handleResponse(cmdConn)
			if err != nil {
				fmt.Println("response parse error:", err)
			}
		}
	}()

	// starts notifications coming from drone to video port normally 11111
	d.SendCommand(d.connectionString())

	// send stick commands
	go func() {
		for {
			err := d.SendStickCommand()
			if err != nil {
				fmt.Println("stick command error:", err)
			}
			time.Sleep(20 * time.Millisecond)
		}
	}()

	return nil
}

// Halt stops the driver.
func (d *Driver) Halt() (err error) {
	// send a landing command when we disconnect, and give it 500ms to be received before we shutdown
	d.Land()
	time.Sleep(500 * time.Millisecond)

	// TODO: cleanly shutdown the goroutines that are handling the UDP connections before closing
	d.cmdConn.Close()
	d.videoConn.Close()
	return
}

// TakeOff tells drones to liftoff and start flying.
func (d *Driver) TakeOff() (err error) {
	return d.sendPacket(takeoffCommand, setType)
}

// Throw & Go support
func (d *Driver) ThrowTakeOff() (err error) {
	return d.sendPacket(throwtakeoffCommand, queryType)
}

// Land tells drone to come in for landing.
func (d *Driver) Land() (err error) {
	return d.sendPacket(landCommand, setType, byte(0))
}

// StopLanding tells drone to stop landing.
func (d *Driver) StopLanding() (err error) {
	return d.sendPacket(landCommand, setType, byte(1))
}

// PalmLand tells drone to come in for a hand landing.
func (d *Driver) PalmLand() (err error) {
	return d.sendPacket(palmLandCommand, setType, byte(0))
}

// StartVideo tells Tello to send start info (SPS/PPS) for video stream.
func (d *Driver) StartVideo() (err error) {
	return d.sendPacket(videoStartCommand, frameType)
}

// SetExposure sets the drone camera exposure level. Valid levels are 0, 1, and 2.
func (d *Driver) SetExposure(level int) (err error) {
	if level < 0 || level > 2 {
		return errors.New("Invalid exposure level")
	}

	return d.sendPacket(exposureCommand, setType, byte(level))
}

// SetVideoEncoderRate sets the drone video encoder rate.
func (d *Driver) SetVideoEncoderRate(rate VideoBitRate) (err error) {
	return d.sendPacket(videoEncoderRateCommand, setType, byte(rate))
}

// SetFastMode sets the drone throttle to 1.
func (d *Driver) SetFastMode() error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.throttle = 1
	return nil
}

// SetSlowMode sets the drone throttle to 0.
func (d *Driver) SetSlowMode() error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.throttle = 0
	return nil
}

// Rate queries the current video bit rate.
func (d *Driver) Rate() (err error) {
	return d.sendPacket(videoRateQuery, queryType)
}

// bound is a naive implementation that returns the smaller of x or y.
func bound(x, y float32) float32 {
	if x < -y {
		return -y
	}
	if x > y {
		return y
	}
	return x
}

// Vector returns the current motion vector.
// Values are from 0 to 1.
// x, y, z denote forward, side and vertical translation,
// and psi  yaw (rotation around the z-axis).
func (d *Driver) Vector() (x, y, z, psi float32) {
	return d.ry, d.rx, d.ly, d.lx
}

// AddVector adds to the current motion vector.
// Pass values from 0 to 1.
// See Vector() for the frame of reference.
func (d *Driver) AddVector(x, y, z, psi float32) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ry = bound(d.ry+x, 1)
	d.rx = bound(d.rx+y, 1)
	d.ly = bound(d.ly+z, 1)
	d.lx = bound(d.lx+psi, 1)

	return nil
}

// SetVector sets the current motion vector.
// Pass values from 0 to 1.
// See Vector() for the frame of reference.
func (d *Driver) SetVector(x, y, z, psi float32) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ry = x
	d.rx = y
	d.ly = z
	d.lx = psi

	return nil
}

// SetX sets the x component of the current motion vector
// Pass values from 0 to 1.
// See Vector() for the frame of reference.
func (d *Driver) SetX(x float32) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ry = x

	return nil
}

// SetY sets the y component of the current motion vector
// Pass values from 0 to 1.
// See Vector() for the frame of reference.
func (d *Driver) SetY(y float32) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.rx = y

	return nil
}

// SetZ sets the z component of the current motion vector
// Pass values from 0 to 1.
// See Vector() for the frame of reference.
func (d *Driver) SetZ(z float32) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ly = z

	return nil
}

// SetPsi sets the psi component (yaw) of the current motion vector
// Pass values from 0 to 1.
// See Vector() for the frame of reference.
func (d *Driver) SetPsi(psi float32) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.lx = psi

	return nil
}

// Up tells the drone to ascend. Pass in an int from 0-100.
func (d *Driver) Up(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ly = float32(val) / 100.0
	return nil
}

// Down tells the drone to descend. Pass in an int from 0-100.
func (d *Driver) Down(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ly = float32(val) / 100.0 * -1
	return nil
}

// Forward tells the drone to go forward. Pass in an int from 0-100.
func (d *Driver) Forward(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ry = float32(val) / 100.0
	return nil
}

// Backward tells drone to go in reverse. Pass in an int from 0-100.
func (d *Driver) Backward(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.ry = float32(val) / 100.0 * -1
	return nil
}

// Right tells drone to go right. Pass in an int from 0-100.
func (d *Driver) Right(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.rx = float32(val) / 100.0
	return nil
}

// Left tells drone to go left. Pass in an int from 0-100.
func (d *Driver) Left(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.rx = float32(val) / 100.0 * -1
	return nil
}

// Clockwise tells drone to rotate in a clockwise direction. Pass in an int from 0-100.
func (d *Driver) Clockwise(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.lx = float32(val) / 100.0
	return nil
}

// CounterClockwise tells drone to rotate in a counter-clockwise direction.
// Pass in an int from 0-100.
func (d *Driver) CounterClockwise(val int) error {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.lx = float32(val) / 100.0 * -1
	return nil
}

// Hover tells the drone to stop moving on the X, Y, and Z axes and stay in place
func (d *Driver) Hover() {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.rx = float32(0)
	d.ry = float32(0)
	d.lx = float32(0)
	d.ly = float32(0)
}

// CeaseRotation stops any rotational motion
func (d *Driver) CeaseRotation() {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	d.lx = float32(0)
}

// Bounce tells drone to start/stop performing the bouncing action
func (d *Driver) Bounce() (err error) {
	pkt := d.nextPacket(bounceCommand, setType)

	if d.bouncing {
		pkt.Write(byte(0x31))
	} else {
		pkt.Write(byte(0x30))
	}

	_, err = d.cmdConn.Write(pkt.Bytes())
	d.bouncing = !d.bouncing
	return
}

// Flip tells drone to flip
func (d *Driver) Flip(direction FlipType) (err error) {
	return d.sendPacket(flipCommand, flipType, byte(direction))
}

// FrontFlip tells the drone to perform a front flip.
func (d *Driver) FrontFlip() (err error) {
	return d.Flip(FlipFront)
}

// BackFlip tells the drone to perform a back flip.
func (d *Driver) BackFlip() (err error) {
	return d.Flip(FlipBack)
}

// RightFlip tells the drone to perform a flip to the right.
func (d *Driver) RightFlip() (err error) {
	return d.Flip(FlipRight)
}

// LeftFlip tells the drone to perform a flip to the left.
func (d *Driver) LeftFlip() (err error) {
	return d.Flip(FlipLeft)
}

// SendStickCommand sends the joystick command packet to the drone.
func (d *Driver) SendStickCommand() (err error) {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	// New packet with seq = 0
	pkt := NewPacketBuffer(stickCommand, frameType, 0)

	// RightX center=1024 left =364 right =-364
	axis1 := int16(660.0*d.rx + 1024.0)

	// RightY down =364 up =-364
	axis2 := int16(660.0*d.ry + 1024.0)

	// LeftY down =364 up =-364
	axis3 := int16(660.0*d.ly + 1024.0)

	// LeftX left =364 right =-364
	axis4 := int16(660.0*d.lx + 1024.0)

	// speed control
	axis5 := int16(d.throttle)

	packedAxis := int64(axis1)&0x7FF | int64(axis2&0x7FF)<<11 | 0x7FF&int64(axis3)<<22 | 0x7FF&int64(axis4)<<33 | int64(axis5)<<44
	pkt.Write(byte(0xFF & packedAxis))
	pkt.Write(byte(packedAxis >> 8 & 0xFF))
	pkt.Write(byte(packedAxis >> 16 & 0xFF))
	pkt.Write(byte(packedAxis >> 24 & 0xFF))
	pkt.Write(byte(packedAxis >> 32 & 0xFF))
	pkt.Write(byte(packedAxis >> 40 & 0xFF))

	now := time.Now()
	pkt.Write(byte(now.Hour()))
	pkt.Write(byte(now.Minute()))
	pkt.Write(byte(now.Second()))
	pkt.Write(byte(now.UnixNano() / int64(time.Millisecond) & 0xff))
	pkt.Write(byte(now.UnixNano() / int64(time.Millisecond) >> 8))

	_, err = d.cmdConn.Write(pkt.Bytes())

	return
}

// SendDateTime sends the current date/time to the drone.
func (d *Driver) SendDateTime() (err error) {
	d.cmdMutex.Lock()
	defer d.cmdMutex.Unlock()

	pkt := d.nextPacket(timeCommand, ackType)

	now := time.Now()
	pkt.Write(byte(0x00))
	pkt.Write(int16(now.Hour()))
	pkt.Write(int16(now.Minute()))
	pkt.Write(int16(now.Second()))
	pkt.Write(int16(now.UnixNano() / int64(time.Millisecond) & 0xff))
	pkt.Write(int16(now.UnixNano() / int64(time.Millisecond) >> 8))

	_, err = d.cmdConn.Write(pkt.Bytes())
	return
}

// SendCommand is used to send a text command such as the initial connection request to the drone.
func (d *Driver) SendCommand(cmd string) (err error) {
	_, err = d.cmdConn.Write([]byte(cmd))
	return
}

func (d *Driver) handleResponse(r io.Reader) error {
	var buf [2048]byte
	var msgType uint16
	n, err := r.Read(buf[0:])
	if err != nil {
		return err
	}

	// parse binary packet
	if buf[0] == messageStart {
		msgType = (uint16(buf[6]) << 8) | uint16(buf[5])
		switch msgType {
		case wifiMessage:
			buf := bytes.NewReader(buf[9:10])
			wd := &WifiData{}
			binary.Read(buf, binary.LittleEndian, &wd.Strength)
			binary.Read(buf, binary.LittleEndian, &wd.Disturb)
			d.Publish(d.Event(WifiDataEvent), wd)
		case lightMessage:
			buf := bytes.NewReader(buf[9:9])
			var ld int8
			binary.Read(buf, binary.LittleEndian, &ld)
			d.Publish(d.Event(LightStrengthEvent), ld)
		case logMessage:
			d.Publish(d.Event(LogEvent), buf[9:])
		case timeCommand:
			d.Publish(d.Event(TimeEvent), buf[7:8])
		case bounceCommand:
			d.Publish(d.Event(BounceEvent), buf[7:8])
		case takeoffCommand:
			d.Publish(d.Event(TakeoffEvent), buf[7:8])
		case landCommand:
			d.Publish(d.Event(LandingEvent), buf[7:8])
		case palmLandCommand:
			d.Publish(d.Event(PalmLandingEvent), buf[7:8])
		case flipCommand:
			d.Publish(d.Event(FlipEvent), buf[7:8])
		case flightMessage:
			fd, _ := ParseFlightData(buf[9:])
			d.Publish(d.Event(FlightDataEvent), fd)
		case exposureCommand:
			d.Publish(d.Event(SetExposureEvent), buf[7:8])
		case videoEncoderRateCommand:
			d.Publish(d.Event(SetVideoEncoderRateEvent), buf[7:8])
		case logDataMessage:
			ld, _ := ParseLogData(buf[10:])
			d.Publish(d.Event(LogDataEvent), ld)
		default:
			fmt.Printf("Unknown message: %+v\n", buf[0:n])
		}
		return nil
	}

	// parse text packet
	if buf[0] == 0x63 && buf[1] == 0x6f && buf[2] == 0x6e {
		d.Publish(d.Event(ConnectedEvent), nil)
	}

	return nil
}

func (d *Driver) processVideo() error {
	videoPort, err := net.ResolveUDPAddr("udp", ":11111")
	if err != nil {
		return err
	}
	d.videoConn, err = net.ListenUDP("udp", videoPort)
	if err != nil {
		return err
	}

	go func() {
		for {
			buf := make([]byte, 2048)
			n, _, err := d.videoConn.ReadFromUDP(buf)
			if err != nil {
				fmt.Println("Error: ", err)
				continue
			}

			d.Publish(d.Event(VideoFrameEvent), buf[2:n])
		}
	}()

	return nil
}

func (d *Driver) connectionString() string {
	x, _ := strconv.Atoi(d.videoPort)
	b := [2]byte{}
	binary.LittleEndian.PutUint16(b[:], uint16(x))
	res := fmt.Sprintf("conn_req:%s", b)
	return res
}

func (d *Driver) nextPacket(cmd uint16, pktType byte, data ...interface{}) *PacketBuffer {
	d.seq++
	return NewPacketBuffer(cmd, pktType, d.seq, data...)
}

func (d *Driver) sendPacket(cmd uint16, pktType byte, data ...interface{}) (err error) {
	d.seq++
	pkt := NewPacket(cmd, pktType, uint16(d.seq), data...)
	_, err = d.cmdConn.Write(pkt.Encode())
	return
}

func (d *Driver) QueryMaxHeight() error {
	return d.sendPacket(maxHeightMessage, queryType)
}

func (d *Driver) QueryAttAngle() error {
	return d.sendPacket(attAngleMessage, queryType)
}

func (d *Driver) SetMaxHeight(h int16) error {
	return d.sendPacket(maxHeightCommand, setType, h)
}

func (d *Driver) SetAttAngle(angle float32) error {
	return d.sendPacket(attAngleCommand, setType, angle)
}

func (d *Driver) SetEIS(v int8) error {
	return d.sendPacket(eisCommand, setType, v)
}

func (d *Driver) SetJPEGQuality(q int8) error {
	return d.sendPacket(jpegQualityCommand, setType, q)
}

func (d *Driver) SetVideoDynamicRate(r int8) error {
	return d.sendPacket(videoDynRateCommand, setType, r)
}

func (d *Driver) SetVideoRecord(n int8) error {
	return d.sendPacket(videoRecordCommand, setType, n)
}

func (d *Driver) SetImageMode(m ImageMode) error {
	return d.sendPacket(picVidModeCommand, setType, byte(m))
}

func (d *Driver) VideoMode() error {
	return d.SetImageMode(VideoMode)
}

func (d *Driver) PhotoMode() error {
	return d.SetImageMode(PictureMode)
}

func (d *Driver) TakePicture() error {
	return d.sendPacket(pictureCommand, setType)
}

func (d *Driver) SendJPEGStartAck() error {
	return d.sendPacket(jpegStartMessage, ackType, byte(0))
}

func (d *Driver) SendJPEGPartAck(endFlag byte, fileID uint16, pieceID uint32) error {
	return d.sendPacket(jpegMessage, ackType, endFlag, fileID, pieceID)
}

func (d *Driver) SendJPEGEndAck(fileID uint16, size int32) error {
	return d.sendPacket(jpegEndMessage, queryType, fileID, size)
}

func (d *Driver) SendLogAck(id uint16) error {
	return d.sendPacket(logMessage, ackType, byte(0), id)
}

func (d *Driver) SendLogConfigAck(id uint16, n2 int32) error {
	return d.sendPacket(logConfigMessage, configType, byte(0), id, n2)
}
