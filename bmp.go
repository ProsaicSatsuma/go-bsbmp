package bsbmp

import (
	"fmt"
	"log"
	"math"
	"os"

	"github.com/d2r2/go-i2c"
)

// SensorType identify which Bosch Sensortec
// temperature and pressure sensor is used.
// BMP180 and BMP280 are supported.
type SensorType int

// Implement Stringer interface.
func (this SensorType) String() string {
	if this == BMP180_TYPE {
		return "BMP180"
	} else if this == BMP280_TYPE {
		return "BMP280"
	} else {
		return "!!! unknown !!!"
	}
}

const (
	// Bosch Sensortec pressure and temperature sensor model BMP180.
	BMP180_TYPE SensorType = iota
	// Bosch Sensortec pressure and temperature sensor model BMP280.
	BMP280_TYPE
)

// Accuracy mode for calculation of atmospheric pressure and temprature.
// Affect to value accuracy, calculation time frame and power consumption.
type AccuracyMode int

const (
	ACCURACY_ULTRA_LOW  AccuracyMode = iota // x1 sample
	ACCURACY_LOW                            // x2 samples
	ACCURACY_STANDARD                       // x4 samples
	ACCURACY_HIGH                           // x8 samples
	ACCURACY_ULTRA_HIGH                     // x16 samples
)

// BMPx sensors memory map
const (
	// General registers
	ID_REG = 0xD0

	// BMP180 general registers
	BMP180_CNTR_MEAS_REG = 0xF4
	BMP180_RESET         = 0xE0
	// BMP180 specific compensation 2-byte registers
	BMP180_COEF_AC1_MSB_LSB = 0xAA
	BMP180_COEF_AC2_MSB_LSB = 0xAC
	BMP180_COEF_AC3_MSB_LSB = 0xAE
	BMP180_COEF_AC4_MSB_LSB = 0xB0
	BMP180_COEF_AC5_MSB_LSB = 0xB2
	BMP180_COEF_AC6_MSB_LSB = 0xB4
	BMP180_COEF_B1_MSB_LSB  = 0xB6
	BMP180_COEF_B2_MSB_LSB  = 0xB8
	BMP180_COEF_MB_MSB_LSB  = 0xBA
	BMP180_COEF_MC_MSB_LSB  = 0xBC
	BMP180_COEF_MD_MSB_LSB  = 0xBE
	BMP180_COEF_START       = BMP180_COEF_AC1_MSB_LSB
	BMP180_COEF_COUNT       = 11
	// BMP180 specific 3-byte reading out temprature and preassure
	BMP180_OUT_MSB_LSB_XLSB = 0xF6

	// BMP280 general registers
	BMP280_STATUS_REG    = 0xF3
	BMP280_CNTR_MEAS_REG = 0xF4
	BMP280_CONFIG        = 0xF5 // TODO: support IIR filter settings
	BMP280_RESET         = 0xE0
	// BMP280 specific compensation 2-byte registers
	BMP280_COEF_T1_LSB_MSB = 0x88
	BMP280_COEF_T2_LSB_MSB = 0x8A
	BMP280_COEF_T3_LSB_MSB = 0x8C
	BMP280_COEF_P1_LSB_MSB = 0x8E
	BMP280_COEF_P2_LSB_MSB = 0x90
	BMP280_COEF_P3_LSB_MSB = 0x92
	BMP280_COEF_P4_LSB_MSB = 0x94
	BMP280_COEF_P5_LSB_MSB = 0x96
	BMP280_COEF_P6_LSB_MSB = 0x98
	BMP280_COEF_P7_LSB_MSB = 0x9A
	BMP280_COEF_P8_LSB_MSB = 0x9C
	BMP280_COEF_P9_LSB_MSB = 0x9E
	BMP280_COEF_START      = BMP280_COEF_T1_LSB_MSB
	BMP280_COEF_COUNT      = 12
	// BMP280 specific 3-byte reading out temprature and preassure
	BMP280_PRESS_OUT_MSB_LSB_XLSB = 0xF7
	BMP280_TEMP_OUT_MSB_LSB_XLSB  = 0xFA
)

// Abstract BMPx sensor interface
// to control and gather data.
type SensorInterface interface {
	SetDebug(debug bool)
	ReadCoefficients(i2c *i2c.I2C) error
	IsValidCoefficients() error
	GetSensorSignature() uint8
	IsBusy(i2c *i2c.I2C) (bool, error)
	ReadTemperatureMult100C(i2c *i2c.I2C, mode AccuracyMode) (int32, error)
	ReadPressureMult10Pa(i2c *i2c.I2C, mode AccuracyMode) (int32, error)
}

// BMP represent both sensors BMP180 and BMP280
// implementing same approach to control and gather data.
type BMP struct {
	sensorType SensorType
	i2c        *i2c.I2C
	bmp        SensorInterface
	// Sensor id
	id uint8
	// Logger
	log *log.Logger
	// Enable verbose output
	debug bool
}

// Create new sensor object.
func NewBMP(sensorType SensorType, i2c *i2c.I2C) (*BMP, error) {
	v := &BMP{sensorType: sensorType, i2c: i2c}
	switch sensorType {
	case BMP180_TYPE:
		v.bmp = &BMP180{log: v.getLogger()}
	case BMP280_TYPE:
		v.bmp = &BMP280{log: v.getLogger()}
	}

	err := v.readSensorID()
	if err != nil {
		return nil, err
	}
	signature := v.bmp.GetSensorSignature()
	if v.id != signature {
		return nil, fmt.Errorf("Sensor id should be 0x%X, but 0x%X received",
			signature, v.id)
	}
	err = v.bmp.ReadCoefficients(i2c)
	if err != nil {
		return nil, err
	}
	return v, nil
}

func (this *BMP) getLogger() *log.Logger {
	if this.log == nil {
		this.log = log.New(os.Stdout, "", log.LstdFlags)
	}
	return this.log
}

func (this *BMP) debugf(format string, args ...interface{}) {
	if this.debug {
		lg := this.getLogger()
		lg.Printf("[bmp] DEBUG "+format, args...)
	}
}

// Read compensation coefficients, which unique for each sensor.
func (this *BMP) readSensorID() error {
	var err error
	this.id, err = this.i2c.ReadRegU8(ID_REG)
	if err != nil {
		return err
	}
	return nil
}

func (this *BMP) SetDebug(debug bool) {
	this.debug = debug
	this.bmp.SetDebug(debug)
}

func (this *BMP) IsValidCoefficients() error {
	return this.bmp.IsValidCoefficients()
}

// Read and calculate temrature in C (celsius).
func (this *BMP) ReadTemperatureMult100C(accuracy AccuracyMode) (int32, error) {
	t, err := this.bmp.ReadTemperatureMult100C(this.i2c, accuracy)
	return t, err
}

// Read and calculate temrature in C (celsius).
func (this *BMP) ReadTemperatureC(accuracy AccuracyMode) (float32, error) {
	t, err := this.bmp.ReadTemperatureMult100C(this.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	return float32(t) / 100, nil
}

// Read and calculate atmospheric pressure in Pa (Pascal).
func (this *BMP) ReadPressureMult10Pa(accuracy AccuracyMode) (int32, error) {
	p, err := this.bmp.ReadPressureMult10Pa(this.i2c, accuracy)
	return p, err
}

// Read and calculate atmospheric pressure in Pa (Pascal).
func (this *BMP) ReadPressurePa(accuracy AccuracyMode) (float32, error) {
	p, err := this.bmp.ReadPressureMult10Pa(this.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	return float32(p) / 10, err
}

// Read and calculate atmospheric pressure in mmHg (millimeter of mercury).
func (this *BMP) ReadPressureMmHg(accuracy AccuracyMode) (float32, error) {
	p, err := this.bmp.ReadPressureMult10Pa(this.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	// Amount of Pa in 1 mmHg
	var mmHg float32 = 133.322
	// Round up to 2 decimals after point
	p2 := float32(int(float32(p)/mmHg*10)) / 100
	return p2, nil
}

// Read and calculate altitude above sea level, if we assume
// that pressure at see level is equal to 101325 Pa.
func (this *BMP) ReadAltitude(accuracy AccuracyMode) (float32, error) {
	p, err := this.bmp.ReadPressureMult10Pa(this.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	// Approximate atmospheric pressure at sea level in Pa
	p0 := 1013250.0
	a := 44330 * (1 - math.Pow(float64(p)/p0, 1/5.255))
	// Round up to 2 decimals after point
	a2 := float32(int(a*100)) / 100
	return a2, nil
}