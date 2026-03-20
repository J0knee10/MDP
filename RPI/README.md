# Instructions to start RPI

## 1. Start Bluetooth connection
bash bluetooth.sh

## 2. Make sure all other server opens and STM connected

## 3. Depending on the simulation or not, run the commands in terminal
make clean

(Have full hardware) make ctrl_center

(No android, have STM, run) make and_center

(No STM, have android, run) make st_center

(No STM, no android, no camera, run) make test_center

## 4. Run the executable
./ctrl_center

./and_center

./st_center

./test_center