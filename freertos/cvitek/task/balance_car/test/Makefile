# Host-side unit tests for the balance-car pure-logic modules.
#   make -C freertos/cvitek/task/balance_car/test        # build + run
#   make ... SAN=1                                         # ASan + UBSan
SRC     := ../src
CC      ?= cc
CFLAGS  += -std=c99 -Wall -Wextra -Werror -O1 -g -DHOST_TEST \
           -D_GNU_SOURCE -Istubs -I../include -I../../../driver/gpio/include
ifdef SAN
CFLAGS  += -fsanitize=address,undefined -fno-omit-frame-pointer
LDFLAGS += -fsanitize=address,undefined
endif
LDLIBS  += -lm -lpthread

FW_SRCS   := $(addprefix $(SRC)/,pid.c encoder.c mpu60x0.c bc_params.c estimator.c control.c \
             bc_state.c imu_health.c bc_cmd.c bc_core.c)
TEST_SRCS := test_main.c test_pid.c test_encoder.c test_mpu.c test_params.c \
             test_estimator.c test_control.c test_state.c test_health_cmd.c \
             test_shm.c test_sim.c sim_plant.c stubs/fake_hw.c
BIN := build/balance_tests

.PHONY: all test clean coverage
all: $(BIN)
test: $(BIN)
	./$(BIN)
$(BIN): $(FW_SRCS) $(TEST_SRCS) $(wildcard *.h stubs/*.h ../include/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) $(FW_SRCS) $(TEST_SRCS) -o $@ $(LDFLAGS) $(LDLIBS)
clean:
	rm -rf build

# Line coverage of the firmware sources under test (gcc + gcov).
# Fails if any file is below COV_MIN percent.
COV_MIN ?= 85
coverage:
	@rm -rf build/cov && mkdir -p build/cov
	$(CC) $(CFLAGS) --coverage -O0 $(FW_SRCS) $(TEST_SRCS) -o build/cov/balance_tests $(LDLIBS)
	cd build/cov && ./balance_tests > run.log && \
	  for f in pid encoder mpu60x0 bc_params estimator control bc_state imu_health bc_cmd bc_core; do \
	    gcov -o . -n balance_tests-$$f.gcda ../../$(SRC)/$$f.c 2>/dev/null | \
	      awk -v f=$$f -v min=$(COV_MIN) '/Lines executed/ && !seen {seen=1; split($$2,a,":"); pct=a[2]+0; \
	        printf "coverage %-8s %5.1f%% (min %d%%)\n", f, pct, min; if (pct < min) bad=1} END {exit bad}' \
	      || exit 1; \
	  done
