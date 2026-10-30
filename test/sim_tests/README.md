# Simulator Tests

These are full-system tests that test out the algorithm nodes inside the simulator


## Run using the following shell script

```bash
cd /racerbot_ws
mkdir -p scripts
#create a symbolic link to actual run_tests.sh
ln -s ../src/racerbot_a/test/sim_tests/run_tests.sh scripts/run_tests.sh
bash ./scripts/run_tests.sh
```



