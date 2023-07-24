package main

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io/ioutil"
	"os"
	"strconv"
	"strings"
)

/*
+---------+---------+-----------+---------+------------------+--------+----------------+------+
| 4 bytes | 1 byte  | 32 bytes  | 4 bytes | 1 byte           | speeds | 4 bytes        | logs |
| AVDR    | Version | Ship name | IMO ID  | Number of speeds | ...    | FF  FF  FF  FF | ...  |
+---------+---------+-----------+---------+------------------+--------+----------------+------+

+--------+---------+
| 1 byte | 7 bytes |
| knots  | Name    |
+--------+---------+

+----------------------+---------------------+-----+----------+----------------+
| 4 bytes              | 1 byte              |     | 1 byte   | 4 bytes        |
| unix time in seconds | event type + length | ... | Checksum | FF  FF  FF  FF |
+----------------------+---------------------+-----+----------+----------------+

Checksum = XOR of the previous bytes in the log (including the unix time, excluding the CRC byte, excluding the 4 last bytes)
*/

func main() {
	reader := bufio.NewReader(os.Stdin)

	fmt.Println("OPTIONS:")
	fmt.Println("1.- New Voyage Data Recorder File")
	option, _ := reader.ReadString('\n')

	switch strings.TrimSpace(option) {
	case "1":
		{
			newVDRFile(reader)
		}
	}
}

func newVDRFile(reader *bufio.Reader) {
	fmt.Println(">>> NEW VOYAGE DATA RECORDER FILE <<<")

	var shipName string
	var IMOString string
	var IMONumber uint32

	for len(shipName) == 0 || len(shipName) > 32 {
		fmt.Println("Enter the name of the ship (max. 32 characters):")
		shipName, _ = reader.ReadString('\n')
		shipName = strings.TrimSpace(shipName)
	}

	for IMONumber == 0 {
		fmt.Println("Enter the ship's IMO Number:")
		IMOString, _ = reader.ReadString('\n')
		IMOString = strings.TrimSpace(IMOString)
		number, _ := strconv.Atoi(IMOString)
		if number > 0 {
			IMONumber = uint32(number)
		}
	}

	var initialData []byte = make([]byte, 0) // 81 bytes long

	// Encode the file type and version
	initialData = append(initialData, []byte{'A', 'V', 'D', 'R'}...) // Arduino Voyage Data Recorder
	initialData = append(initialData, 0x01)                          // Version 1

	// Encode the ship's name
	var shipNameEncoded []byte = []byte(strings.ToUpper(shipName))
	for len(shipNameEncoded) < 32 {
		shipNameEncoded = append(shipNameEncoded, ' ')
	}
	initialData = append(initialData, shipNameEncoded...)

	// Encode the ship's IMO Number
	var IMONumberEncoded []byte = make([]byte, 4)
	binary.LittleEndian.PutUint32(IMONumberEncoded, IMONumber)
	initialData = append(initialData, IMONumberEncoded...)

	// Encode hard-coded data: 5 speeds
	var hardcodedSpeedData []byte = []byte{5, 2, 'D', 'E', 'A', 'D', 'S', 'L', 'W', 5, 'S', 'L', 'O', 'W', ' ', ' ', ' ', 10, 'H', 'A', 'L', 'F', ' ', ' ', ' ', 20, 'F', 'U', 'L', 'L', ' ', ' ', ' ', 22, 'F', 'L', 'A', 'N', 'K', ' ', ' '}
	initialData = append(initialData, hardcodedSpeedData...)

	// Write the terminator: 4 bytes of ones
	initialData = append(initialData, 0xFF)
	initialData = append(initialData, 0xFF)
	initialData = append(initialData, 0xFF)
	initialData = append(initialData, 0xFF)

	ioutil.WriteFile("voyage_data_recorder.dat", initialData, 0777)
}
