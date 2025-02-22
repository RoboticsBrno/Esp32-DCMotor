from serial import Serial
import sys
import time

TestStep = tuple[int, str]


def echo(ser: Serial):
    data = ser.read_all().decode()
    if data:
        sys.stderr.write(data)


def main() -> None:
    if len(sys.argv) != 4:
        print("Usage: python3 run_test.py <port> <test_file> <out_file>")
        exit(1)

    port = sys.argv[1]
    test_file = sys.argv[2]
    out_file = sys.argv[3]

    steps: list[TestStep] = []
    with open(test_file, "r") as file:
        for line in file:
            time_point, command = line.strip().split(":", 1)
            steps.append((int(time_point.strip()), command.strip()))

    get_reports = "report l p"

    with Serial(port, 115200, timeout=1) as ser:
        start = time.time()
        for step in steps:
            while time.time() - start < step[0] / 1000:
                echo(ser)
                pass
            ser.write((step[1] + "\n").encode())
            echo(ser)

        time.sleep(0.5)
        echo(ser)

        ser.write((get_reports + "\n").encode())
        time.sleep(0.1)

        output: list[bytes] = []
        while True:
            data = ser.read_all()
            if data:
                output.append(data)
                time.sleep(0.1)
            else:
                break

    out_str = b"".join(output).decode()

    reports = out_str.split("\n")[5:-5]

    with open(out_file, "w") as file:
        print("Writing", len(reports), "lines")
        for report in reports:
            file.write(report.strip("\r") + "\n")


if __name__ == "__main__":
    main()
