package tello

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"io/ioutil"
	"testing"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/gobottest"
)

var _ gobot.Driver = (*Driver)(nil)

type testBuffer struct {
	bytes.Buffer
}

func (tb *testBuffer) Close() error {
	tb.Buffer = bytes.Buffer{}
	return nil
}

func TestTelloDriver(t *testing.T) {
	d := NewDriver("8888")

	gobottest.Assert(t, d.respPort, "8888")
}

func statusMessage(msgType uint16, msgAfter7 ...byte) []byte {
	msg := make([]byte, 7, len(msgAfter7)+7)
	msg[0] = messageStart
	binary.LittleEndian.PutUint16(msg[5:7], msgType)
	msg = append(msg, msgAfter7...)
	return msg
}

func TestHandleResponse(t *testing.T) {
	cc := []struct {
		name   string
		msg    io.Reader
		events []gobot.Event
		err    error
	}{
		{
			name: "[empty messsage]",
			msg:  bytes.NewReader(nil),
			err:  io.EOF,
		},
		{
			name:   "wifiMessage",
			msg:    bytes.NewReader(statusMessage(wifiMessage)),
			events: []gobot.Event{{Name: WifiDataEvent}},
		},
		{
			name:   "lightMessage",
			msg:    bytes.NewReader(statusMessage(lightMessage)),
			events: []gobot.Event{{Name: LightStrengthEvent}},
		},
		{
			name:   "logMessage",
			msg:    bytes.NewReader(statusMessage(logMessage)),
			events: []gobot.Event{{Name: LogEvent}},
		},
		{
			name:   "timeCommand",
			msg:    bytes.NewReader(statusMessage(timeCommand)),
			events: []gobot.Event{{Name: TimeEvent}},
		},
		{
			name:   "bounceCommand",
			msg:    bytes.NewReader(statusMessage(bounceCommand)),
			events: []gobot.Event{{Name: BounceEvent}},
		},
		{
			name:   "takeoffCommand",
			msg:    bytes.NewReader(statusMessage(takeoffCommand)),
			events: []gobot.Event{{Name: TakeoffEvent}},
		},
		{
			name:   "landCommand",
			msg:    bytes.NewReader(statusMessage(landCommand)),
			events: []gobot.Event{{Name: LandingEvent}},
		},
		{
			name:   "palmLandCommand",
			msg:    bytes.NewReader(statusMessage(palmLandCommand)),
			events: []gobot.Event{{Name: PalmLandingEvent}},
		},
		{
			name:   "flipCommand",
			msg:    bytes.NewReader(statusMessage(flipCommand)),
			events: []gobot.Event{{Name: FlipEvent}},
		},
		{
			name:   "flightMessage",
			msg:    bytes.NewReader(statusMessage(flightMessage)),
			events: []gobot.Event{{Name: FlightDataEvent}},
		},
		{
			name:   "exposureCommand",
			msg:    bytes.NewReader(statusMessage(exposureCommand)),
			events: []gobot.Event{{Name: SetExposureEvent}},
		},
		{
			name:   "videoEncoderRateCommand",
			msg:    bytes.NewReader(statusMessage(videoEncoderRateCommand)),
			events: []gobot.Event{{Name: SetVideoEncoderRateEvent}},
		},
		{
			name:   "ConnectedEvent",
			msg:    bytes.NewReader([]byte{0x63, 0x6f, 0x6e}),
			events: []gobot.Event{{Name: ConnectedEvent}},
		},
	}

	for _, c := range cc {
		t.Run(c.name, func(t *testing.T) {
			d := NewDriver("8888")
			events := d.Subscribe()
			err := d.handleResponse(c.msg)
			if c.err != err {
				t.Errorf("expected '%v' error, got: %v", c.err, err)
			}
			for i, cev := range c.events {
				t.Run(fmt.Sprintf("event %d", i), func(t *testing.T) {
					t.Logf("expect: %#v", cev)
					select {
					case ev, ok := <-events:
						if !ok {
							t.Error("subscription channel is closed")
						}
						if ev.Name != cev.Name {
							t.Errorf("got: %s", ev.Name)
						}
					case <-time.After(time.Millisecond):
						t.Error("subscription channel seems empty")
					}
				})
			}
		})
	}
}

func TestNewPacket(t *testing.T) {
	pkt := NewPacket(takeoffCommand, setType, 1231)
	pkt.Write(float32(1))
	pkt.Write(float32(2))
	pkt.Write(float32(2))
	pkt.Write(float32(4))

	if bytes.Equal(pkt.Data, pkt.buf.Bytes()) {
		t.Errorf("%x != %x", pkt.Data, pkt.buf.Bytes())
	}
}

func BenchmarkPacket(b *testing.B) {
	for i := 0; i < b.N; i++ {
		pkt := NewPacket(takeoffCommand, setType, 1231)
		pkt.Write(float32(1))
		pkt.Write(float32(2))
		pkt.Write(float32(2))
		pkt.Write(float32(4))
		pkt.Encode()
	}
}

func BenchmarkPacketBuffer(b *testing.B) {
	for i := 0; i < b.N; i++ {
		pkt := NewPacketBuffer(takeoffCommand, setType, 1231)
		pkt.Write(float32(1))
		pkt.Write(float32(2))
		pkt.Write(float32(2))
		pkt.Write(float32(4))
		pkt.Bytes()
	}
}

type testVector struct {
	f func(*Driver) error
	v []byte
}

var testVectors = map[string]testVector{
	"TakeOff": {
		(*Driver).TakeOff,
		[]byte{0xcc, 0x58, 0x00, 0x7c, 0x68, 0x54, 0x00, 0x09, 0x00},
	},
	"ThrowTakeOff": {
		(*Driver).ThrowTakeOff,
		[]byte{0xcc, 0x58, 0x00, 0x7c, 0x48, 0x5d, 0x00, 0x09, 0x00},
	},
	"Land": {
		(*Driver).Land,
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x55, 0x00, 0x09, 0x00, 0x00},
	},
	"StopLanding": {
		(*Driver).StopLanding,
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x55, 0x00, 0x09, 0x00, 0x01},
	},
	"PalmLand": {
		(*Driver).PalmLand,
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x5e, 0x00, 0x09, 0x00, 0x00},
	},
	"StartVideo": {
		(*Driver).StartVideo,
		[]byte{0xcc, 0x58, 0x00, 0x7c, 0x60, 0x25, 0x00, 0x09, 0x00},
	},
	"SetExposure": {
		func(d *Driver) error { return d.SetExposure(0) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x34, 0x00, 0x09, 0x00, 0x00},
	},
	"SetVideoEncoderRate": {
		func(d *Driver) error { return d.SetVideoEncoderRate(VideoBitRate3M) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x20, 0x00, 0x09, 0x00, 0x04},
	},
	"Rate": {
		(*Driver).Rate,
		[]byte{0xcc, 0x58, 0x0, 0x27, 0x48, 0x28, 0x0, 0x09, 0x0},
	},
	"Bounce": {
		(*Driver).Bounce,
		[]byte{0xcc, 0x60, 0x0, 0x27, 0x68, 0x53, 0x10, 0x09, 0x0, 0x30},
	},
	"Flip": {
		func(d *Driver) error { return d.Flip(FlipForwardLeft) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x70, 0x5c, 0x00, 0x09, 0x00, 0x04},
	},
	"SendStickCommand": {
		(*Driver).SendStickCommand,
		[]byte{0xcc, 0xb0, 0x00, 0x7f, 0x60, 0x50, 0x00, 0x00, 0x00, 0x0, 0x4, 0x20, 0x0, 0x1, 0x8, 0x15, 0x19, 0x1d, 0x3b, 0xfb},
	},
	"SendDateTime": {
		(*Driver).SendDateTime,
		[]byte{0xcc, 0xb0, 0x0, 0x7f, 0x50, 0x46, 0x0, 0x9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
	},
	"QueryMaxHeight": {
		(*Driver).QueryMaxHeight,
		[]byte{0xcc, 0x58, 0x00, 0x7c, 0x48, 0x56, 0x10, 0x09, 0x00},
	},
	"QueryAttAngle": {
		(*Driver).QueryAttAngle,
		[]byte{0xcc, 0x58, 0x00, 0x7c, 0x48, 0x59, 0x10, 0x09, 0x00},
	},
	"SetMaxHeight": {
		func(d *Driver) error { return d.SetMaxHeight(0) },
		[]byte{0xcc, 0x68, 0x00, 0x27, 0x68, 0x58, 0x00, 0x09, 0x00, 0x00, 0x00},
	},
	"SetAttAngle": {
		func(d *Driver) error { return d.SetAttAngle(0) },
		[]byte{0xcc, 0x78, 0x00, 0x27, 0x68, 0x58, 0x10, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
	"SetEIS": {
		func(d *Driver) error { return d.SetEIS(0) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x24, 0x00, 0x09, 0x00, 0x00},
	},
	"SetJPEGQuality": {
		func(d *Driver) error { return d.SetJPEGQuality(0) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x37, 0x00, 0x09, 0x00, 0x00},
	},
	"SetVideoDynamicRate": {
		func(d *Driver) error { return d.SetVideoDynamicRate(0) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x21, 0x00, 0x09, 0x00, 0x00},
	},
	"SetVideoRecord": {
		func(d *Driver) error { return d.SetVideoRecord(0) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x32, 0x00, 0x09, 0x00, 0x00},
	},
	"SetImageMode": {
		func(d *Driver) error { return d.SetImageMode(0) },
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x68, 0x31, 0x00, 0x09, 0x00, 0x00},
	},
	"TakePicture": {
		(*Driver).TakePicture,
		[]byte{0xcc, 0x58, 0x00, 0x7c, 0x68, 0x30, 0x00, 0x09, 0x00},
	},
	"SendJPEGStartAck": {
		(*Driver).SendJPEGStartAck,
		[]byte{0xcc, 0x60, 0x00, 0x27, 0x50, 0x62, 0x00, 0x09, 0x00, 0x00},
	},
	"SendJPEGPartAck": {
		func(d *Driver) error { return d.SendJPEGPartAck(0, 0, 0) },
		[]byte{0xcc, 0x90, 0x00, 0x27, 0x50, 0x63, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
	"SendJPEGEndAck": {
		func(d *Driver) error { return d.SendJPEGEndAck(0, 0) },
		[]byte{0xcc, 0x88, 0x00, 0x24, 0x48, 0x64, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
	"SendLogAck": {
		func(d *Driver) error { return d.SendLogAck(0) },
		[]byte{0xcc, 0x70, 0x00, 0x27, 0x50, 0x50, 0x10, 0x09, 0x00, 0x00, 0x00, 0x00},
	},
	"SendLogConfigAck": {
		func(d *Driver) error { return d.SendLogConfigAck(0, 0) },
		[]byte{0xcc, 0x90, 0x00, 0x27, 0x88, 0x52, 0x10, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
}

func TestVectors(t *testing.T) {
	d := &Driver{
		cmdConn: &testBuffer{},
	}
	for n, test := range testVectors {
		d.seq = 8

		// Run test function
		err := test.f(d)
		if err != nil {
			t.Errorf("%s error: %s", n, err)
			continue
		}

		// Decode sent packet
		p := new(Packet)
		b, _ := ioutil.ReadAll(d.cmdConn)
		p, err = DecodePacket(b)
		if err != nil {
			t.Errorf("%s packet error: %s", n, err)
			t.FailNow()
			continue
		}

		// Ignore timestamps in comparison with test vector
		if n == "SendStickCommand" {
			copy(p.Data[len(p.Data)-5:], test.v[len(test.v)-5:])
		} else if n == "SendDateTime" {
			copy(p.Data[1:], test.v[len(test.v)-10:])
		}

		// Ignore CRC8 in comparison with test vector
		b[3] = test.v[3]

		// Compare contents without CRC16
		if !bytes.Equal(b[0:len(b)-2], test.v) {
			tp, _ := DecodePacket(append(test.v, 0x00, 0x00))
			t.Errorf("%s:\nexpected %#v\ngot      %#v", n, tp, p)
			continue
		}
	}
}
