# Convenience targets for Mecanum AMR (Tarik Kahraman).
# Requires: source install/setup.bash for run/test targets.

.PHONY: build test clean deps stm32-host-test

build:
	colcon build --symlink-install

test: build
	. install/setup.bash && colcon test --return-code-on-test-failure

stm32-host-test:
	$(MAKE) -C hardware/stm32 build_host
	./hardware/stm32/main_host

clean:
	rm -rf build install log
	$(MAKE) -C hardware/stm32 clean 2>/dev/null || true

deps:
	rosdep update && rosdep install --from-paths src --ignore-src -r -y
