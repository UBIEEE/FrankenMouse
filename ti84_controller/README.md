# TI-84 Plus CE MicroMouse Controller

For fun, the MicroMouse may be controlled using a TI-84 Plus CE calculator.
This folder contains the code for the calculator-side program, which is written in C.
The program is built using the [CE C/C++ Toolchain](https://ce-programming.github.io/toolchain/) with the [srldrvce](https://ce-programming.github.io/toolchain/libraries/srldrvce.html) library.

The program reads the calculator's buttons and sends control data to the [CLI](../cli/README.md) over USB.

## Building & Installing

Install the [CE C/C++ Toolchain](https://ce-programming.github.io/toolchain/).

Build the program using `make`:
```bash
make
```

Now transfer the `bin/MOUSE.8xp` file to the TI-84 Plus CE calculator using [TI-Connect CE](https://education.ti.com/en/products/computer-software/ti-connect-ce-sw) or [tilp](http://tilp.info).

Also, install the [CE Libraries](https://github.com/CE-Programming/libraries/releases/latest) on the calculator if they are not already installed.

## Usage

> [!NOTE]
> On OS versions 5.5.0 and greater, the TI-84 Plus CE calculator no longer supports running native assembly programs by default.
>
> If your calculator is running OS version 5.5.0 or greater, you must use [arTIfiCE](https://yvantt.github.io/arTIfiCE/) to restore this functionality.

Connect the calculator to the computer using a USB cable.

Run the `MOUSE` program on the calculator.

On the computer, run the [shell](../cli/README.md) and execute the `ti84-control` command to facilitate communication between the calculator and the MicroMouse. Specify the serial port to which the calculator is connected (e.g., `/dev/ttyUSB0` on Linux, or `/dev/cu.usbmodemXXXX` on macOS).

```bash
> ti84-control /dev/ttyUSB0
```
