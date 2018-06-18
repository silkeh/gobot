package tello

import (
	"bytes"
	"encoding/binary"
)

// PacketBuffer represents a buffer for the creation of a Tello data packet
type PacketBuffer struct {
	*bytes.Buffer
}

// NewPacketBuffer creates a new PacketBuffer
func NewPacketBuffer(cmd uint16, pktType byte, seq uint16, data ...interface{}) *PacketBuffer {
	// PacketBuffer header: start, length, length, header CRC, type
	b := []byte{messageStart, 0, 0, 0, pktType}

	// Write rest of header
	buf := bytes.NewBuffer(b)
	binary.Write(buf, binary.LittleEndian, cmd)
	binary.Write(buf, binary.LittleEndian, seq)

	// Add data
	for _, d := range data {
		binary.Write(buf, binary.LittleEndian, d)
	}

	return &PacketBuffer{Buffer: buf}
}

// WriteData writes and arbitrary amount of arbitrary data using the correct encoding.
func (p *PacketBuffer) Write(data ...interface{}) *PacketBuffer {
	for _, d := range data {
		binary.Write(p.Buffer, binary.LittleEndian, d)
	}
	return p
}

// Bytes returns the bytes of the packet for transmission.
// The PacketBuffer should not be used after calling this function.
func (p *PacketBuffer) Bytes() []byte {
	// Calculate length
	l := uint16(p.Len()) + 2
	b := p.Buffer.Bytes()

	// Set correct header info
	binary.LittleEndian.PutUint16(b[1:3], l<<3)
	b[3] = CalculateCRC8(b[0:3])

	// Write CRC16
	binary.Write(p.Buffer, binary.LittleEndian, CalculateCRC16(b))

	return p.Buffer.Bytes()
}
