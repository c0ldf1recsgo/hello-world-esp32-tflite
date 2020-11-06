# This will show you how to make Harmonic Motion Led deployed on ESP32 using Deep Learning from Tensorflow
This was created by following the intructions on Tensorflow's Github.
This can cause some errors for your machine.
Be careful! Take it at your own risk.

### Some requirements:
- [Install python3 and pip](#install-python3-and-pip)
- [Some important packages](#some-important-packages)
- [ESP-IDF](#esp-idf)
- [Conda](#conda) (Still not working)

### Start working
- [Cloning the repository](#clone-repository)
- [Generate Example Project](#generate-example-project)
- [Load and run the example](#load-and-run-the-example)
- [Training model](#training-model)
- [Working with ESP32 module](#working-with-esp32-module)
- [Demo](#demo)

## Install python3 and pip
Open the terminal and enter
```python
$ sudo apt-get install python-is-python3
$ sudo install python3-setuptools
$ sudo install python3-pip
```
## Some important packages
Install those if you did't have them yet
```
$ sudo apt-get install curl git wget flex bison gperf cmake ninja-build ccache libffi-dev libssl-dev dfu-util
```
## ESP-IDF
Follow the instructions of the
[ESP-IDF get started guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
to setup the toolchain and the ESP-IDF itself.

The next steps assume that the
[IDF environment variables are set](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html#step-4-set-up-the-environment-variables)

## Conda
~~* Note: This is not important unless you want to work on a virtual environment~~
* But sadly, I still don't know how to use it with ESP-IDF, so it is not recommended to use this

~~Follow the guide on this website:~~
~~[Install Conda on Linux](https://docs.anaconda.com/anaconda/install/linux/)~~

~~Next: Create the environment with tensorflow and python3 using Terminal~~
~~```~~
~~$ conda create -n mytf tensorflow python=3~~
~~```~~
~~Now you should have your own Virtual Environment~~

## Cloning the repository
- Get into a folder of your choice. Ex: **Desktop/**
- Enter the command in terminal:
```
$ git clone https://github.com/tensorflow/tensorflow.git
```
- Change to the 2.2.0 version for better performance

You must get to the folder that contains **.git** folder
```
$ cd tensorflow
$ git checkout v2.2.0
```

## Generate Example Project
- Enter the command into your Terminal:
```
$ make -f tensorflow/lite/micro/tools/make/Makefile TARGET=esp TAGS=no_arc_mli generate_hello_world_esp_project
```
- If you get this error:
*make: \*\*\* No rule to make target \[...], needed by \[...sdkconfig.defaults].  Stop.*
- Just add an empty file
```
$ touch tensorflow/lite/micro/examples/hello_world/esp/sdkconfig.defaults
```
- Then build it 
```
$ cd tensorflow/lite/micro/tools/make/gen/esp_xtensa-esp32/prj/hello_world/esp-idf
$ idf.py build
```

- After a succesfully build, you will have this near the end of your terminal:
> Project build complete. To flash, run this command:
> 
> .
>
> .
>
> .

## Load and run the example

- To flash (replace `/dev/ttyUSB0` with the device serial port):
```
idf.py --port /dev/ttyUSB0 flash
```

- Monitor the serial output:
```
idf.py --port /dev/ttyUSB0 monitor
```

- Use `Ctrl+]` to exit.

- The previous two commands can be combined:
```
idf.py --port /dev/ttyUSB0 flash monitor
```

## Training model

- The setting up is complete. Now we will mainly focus on **main** folder in **tensorflow/lite/micro/tools/make/gen/esp_xtensa-esp32/prj/hello_world/esp-idf** folder we'd build before.

### Training a sinusoid model and create Harmonic Motion
- At this step, you can train a model that's called *sinusoid* model using anything you want and import the `.cc` model file into the **main** folder above.
- But the **easiest** way to do that is using the projects that are already made for everyone.
- Like the one on **TensorFlow** and it was built on **Google Colab**.
- Follow this Link: [Train a basic TensorFlow Lite for Microcontrollers model](https://colab.research.google.com/github/tensorflow/tensorflow/blob/master/tensorflow/lite/micro/examples/hello_world/train/train_hello_world_model.ipynb)

### Import the built model to the project
- Rename the file `model.cc` that was created from the previous step into `sine_model_data.cc`.
- Copy it and paste into the **main** folder.

## Working with ESP32 module

**NOTE:** This step is very important to make the module run correctly.
- Open `main_functions.cc` in `tensorflow/lite/micro/tools/make/gen/esp_xtensa-esp32/prj/hello_world/esp-idf` folder above
- Import library:
  - <stdio.h> is not imported in the default `main_functions.cc` file
  - We will need `"freertos/FreeRTOS.h"` and `"freertos/task.h"`
  - Finally, we need logical handlers GPIO to work with the PINOUT on ESP32 Module using `"driver/gpio.h"`
  ```c
  #include <stdio.h>
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "driver/gpio.h"
  ```
- Create an array of GPIO, replace the number for any port you want, I use 8 PIN so mine is:
```c
gpio_num_t pinArray[8] = {GPIO_NUM_0, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_19};
```
- Add some global variables:
```c
float x_val = M_PI / 2;
float y_val;
float dt = 2 * M_PI / SEGMENT;
```

- The following is Setting up GPIO
```c
void setupGPIO() {
  gpio_config_t gpioConfig;
  
  gpioConfig.pin_bit_mask = 0;
  
  //8 Pin
  for (int i = 0; i < 8; i++) {
    gpioConfig.pin_bit_mask |= (1 << pinArray[i]);
  }

  gpioConfig.mode = GPIO_MODE_OUTPUT;
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&gpioConfig);
}
```
- Create a function to decide which LED will go on
  Each LED is selected with values that divided equally in the range [-1, 1]
  Return -1 (No LED chosen because VALUE Y IS INVALID)
```c
int chooseLED(float y) {
  if (-1 <= y && y < -0.75) return 0;
  if (-0.75 <= y && y < -0.5) return 1;
  if (-0.5 <= y && y < -0.25) return 2;
  if (-0.25 <= y && y < 0) return 3;
  if (0 <= y && y < 0.25) return 4;
  if (0.25 <= y && y <= 0.5) return 5;
  if (0.5 <= y && y <= 0.75) return 6;
  if (0.75 <= y && y <= 1) return 7;
  return -1;
}
```
 
- Load the model we've trained, put the `setupGPIO()` at the end.
```c
void setup() {
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  
  model = tflite::GetModel(g_sine_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                        "Model provided is schema version %d not equal "
                        "to supported version %d.",
                        model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }
  
  static tflite::ops::micro::AllOpsResolver resolver;

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  inference_count = 0;
  
  setupGPIO();
}
```

- In the `loop`function, add this at the beginning to make x_val right:
```c
if (x_val > 2 * M_PI) x_val -= 2 * M_PI;
```

- Then do this before *Increment the inference_counter* to select the LED with `y_val` to light up in 1000ms
  You can replace 1000 with any number
```c
for(int i = 0; i < 8; i++) {
    gpio_set_level(pinArray[i], 0);
  }
int iLED = chooseLED(y_val);
gpio_set_level(pinArray[iLED], 1);
vTaskDelay((1000 / SEGMENT) / portTICK_PERIOD_MS);
gpio_set_level(pinArray[iLED], 0);
```

- Update `x_val` by `dt`: 
```c
x_val += dt;
```

- Then *Increment the inference_counter* as the default

## Demo
- See this demo on youtube [Demo](https://youtu.be/EFfrAB1IC54)
- Circuit Board

![Circuit](https://github.com/c0ldf1recsgo/hello-world-esp32-tflite/blob/master/image/tinkercad.png)

- Real board

![Real](https://github.com/c0ldf1recsgo/hello-world-esp32-tflite/blob/master/image/real.jpg)
