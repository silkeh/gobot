package tello

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"time"
)

// FlightData packet returned by the Tello
type FlightData struct {
	BatteryLow               bool
	BatteryLower             bool
	BatteryPercentage        int8
	BatteryState             bool
	CameraState              int8
	DownVisualState          bool
	DroneBatteryLeft         int16
	DroneFlyTimeLeft         int16
	DroneHover               bool
	EmOpen                   bool
	Flying                   bool
	OnGround                 bool
	EastSpeed                int16
	ElectricalMachineryState int16
	FactoryMode              bool
	FastMode                 bool
	FlyMode                  int8
	FlyTime                  int16
	FrontIn                  bool
	FrontLSC                 bool
	FrontOut                 bool
	GravityState             bool
	VerticalSpeed            int16
	Height                   int16
	ImuCalibrationState      int8
	ImuState                 bool
	LightStrength            int8
	NorthSpeed               int16
	OutageRecording          bool
	PowerState               bool
	PressureState            bool
	SmartVideoExitMode       int16
	TemperatureHigh          bool
	ThrowFlyTimer            int8
	WindState                bool
}

// ParseFlightData from drone
func ParseFlightData(b []byte) (fd *FlightData, err error) {
	buf := bytes.NewReader(b)
	fd = &FlightData{}
	var data byte

	if buf.Len() < 24 {
		err = errors.New("invalid buffer length for flight data packet")
		fmt.Println(err)
		return
	}

	err = binary.Read(buf, binary.LittleEndian, &fd.Height)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.NorthSpeed)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.EastSpeed)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.VerticalSpeed)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.FlyTime)
	if err != nil {
		return
	}

	err = binary.Read(buf, binary.LittleEndian, &data)
	if err != nil {
		return
	}
	fd.ImuState = (data >> 0 & 0x1) == 1
	fd.PressureState = (data >> 1 & 0x1) == 1
	fd.DownVisualState = (data >> 2 & 0x1) == 1
	fd.PowerState = (data >> 3 & 0x1) == 1
	fd.BatteryState = (data >> 4 & 0x1) == 1
	fd.GravityState = (data >> 5 & 0x1) == 1
	fd.WindState = (data >> 7 & 0x1) == 1

	err = binary.Read(buf, binary.LittleEndian, &fd.ImuCalibrationState)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.BatteryPercentage)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.DroneFlyTimeLeft)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.DroneBatteryLeft)
	if err != nil {
		return
	}

	err = binary.Read(buf, binary.LittleEndian, &data)
	if err != nil {
		return
	}
	fd.Flying = (data >> 0 & 0x1) == 1
	fd.OnGround = (data >> 1 & 0x1) == 1
	fd.EmOpen = (data >> 2 & 0x1) == 1
	fd.DroneHover = (data >> 3 & 0x1) == 1
	fd.OutageRecording = (data >> 4 & 0x1) == 1
	fd.BatteryLow = (data >> 5 & 0x1) == 1
	fd.BatteryLower = (data >> 6 & 0x1) == 1
	fd.FactoryMode = (data >> 7 & 0x1) == 1

	err = binary.Read(buf, binary.LittleEndian, &data)
	if err != nil {
		return
	}

	fd.FlyMode = int8(data)
	fd.FastMode = (data >> 4 & 0x1) == 1

	err = binary.Read(buf, binary.LittleEndian, &fd.ThrowFlyTimer)
	if err != nil {
		return
	}
	err = binary.Read(buf, binary.LittleEndian, &fd.CameraState)
	if err != nil {
		return
	}

	err = binary.Read(buf, binary.LittleEndian, &data)
	if err != nil {
		return
	}
	fd.ElectricalMachineryState = int16(data & 0xff)

	err = binary.Read(buf, binary.LittleEndian, &data)
	if err != nil {
		return
	}
	fd.FrontIn = (data >> 0 & 0x1) == 1
	fd.FrontOut = (data >> 1 & 0x1) == 1
	fd.FrontLSC = (data >> 2 & 0x1) == 1

	err = binary.Read(buf, binary.LittleEndian, &data)
	if err != nil {
		return
	}
	fd.TemperatureHigh = (data >> 0 & 0x1) == 1

	return
}

func (f *FlightData) AirSpeed() float64 {
	return math.Sqrt(
		math.Pow(float64(f.NorthSpeed), 2) +
			math.Pow(float64(f.EastSpeed), 2) +
			math.Pow(float64(f.VerticalSpeed), 2))
}

func (f *FlightData) GroundSpeed() float64 {
	return math.Sqrt(
		math.Pow(float64(f.NorthSpeed), 2) +
			math.Pow(float64(f.EastSpeed), 2))
}

func (f *FlightData) FlightDuration() time.Duration {
	return time.Duration(f.FlyTime) * 100 * time.Millisecond
}

func (f *FlightData) ThrowTimer() *time.Timer {
	return time.NewTimer(time.Duration(f.ThrowFlyTimer) * 100 * time.Millisecond)
}
