package tello

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
)

// Packet represents a Tello network packet
type Packet struct {
	Len     uint16
	CRC8    uint8
	Type    uint8
	Command uint16
	Seq     uint16
	Data    []byte
	CRC16   uint16
	buf     *bytes.Buffer
}

// DecodePacket decodes a binary packet and returns a Packet.
func DecodePacket(b []byte) (p *Packet, err error) {
	if len(b) == 0 || b[0] != messageStart {
		err = errors.New("invalid packet")
		return
	}

	l := binary.LittleEndian.Uint16(b[1:3]) >> 3
	if len(b) < int(l) || l < 11 {
		err = errors.New(fmt.Sprintf("invalid length, expected %v but got %v", len(b), l))
		return
	}

	p = &Packet{
		Len:     l,
		CRC8:    b[3],
		Type:    b[4],
		Command: binary.LittleEndian.Uint16(b[5:7]),
		Seq:     binary.LittleEndian.Uint16(b[7:9]),
		Data:    b[9 : l-2],
		CRC16:   binary.LittleEndian.Uint16(b[l-2:]),
	}

	if crc16 := CalculateCRC16(b[:l-2]); p.CRC16 != crc16 {
		err = errors.New(fmt.Sprintf("crc16 mismatch, expected %x but got %x", crc16, p.CRC16))
		return
	}
	if crc8 := CalculateCRC8(b[0:3]); p.CRC8 != crc8 {
		err = errors.New(fmt.Sprintf("crc8 mismatch, expected %x but got %x", crc8, p.CRC8))
		return
	}

	return
}

// NewPacket creates a new Packet which can be used for transmission.
func NewPacket(cmd uint16, pktType byte, seq uint16, data ...interface{}) *Packet {
	p := &Packet{
		Type:    pktType,
		Command: cmd,
		Seq:     seq,
	}

	p.buf = &bytes.Buffer{}
	for _, d := range data {
		binary.Write(p.buf, binary.LittleEndian, d)
	}

	p.Data = p.buf.Bytes()
	p.Len = uint16(len(p.Data)) + 11

	return p
}

// Encode returns the binary encoded data for the Packet.
func (p *Packet) Encode() []byte {
	buf := &bytes.Buffer{}

	binary.Write(buf, binary.LittleEndian, byte(messageStart))
	binary.Write(buf, binary.LittleEndian, p.Len<<3)

	p.CRC8 = CalculateCRC8(buf.Bytes())
	binary.Write(buf, binary.LittleEndian, p.CRC8)
	binary.Write(buf, binary.LittleEndian, p.Type)
	binary.Write(buf, binary.LittleEndian, p.Command)
	binary.Write(buf, binary.LittleEndian, p.Seq)
	binary.Write(buf, binary.LittleEndian, p.Data)

	p.CRC16 = CalculateCRC16(buf.Bytes())
	binary.Write(buf, binary.LittleEndian, p.CRC16)

	return buf.Bytes()
}

// Write adds arbitrary data to the packet using the correct encoding.
func (p *Packet) Write(data ...interface{}) *Packet {
	if p.buf == nil {
		p.buf = bytes.NewBuffer(p.Data)
	}

	for _, d := range data {
		binary.Write(p.buf, binary.LittleEndian, d)
	}

	p.Len = uint16(p.buf.Len()) + 11

	return p
}
