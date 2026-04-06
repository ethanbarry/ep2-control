PYTHON ?= python3

.PHONY: install setup-linux setup-macos run-demo run-failure-demo run-live-3d test

install:
	$(PYTHON) -m venv .venv
	. .venv/bin/activate && python -m pip install --upgrade pip && python -m pip install -r requirements.txt

setup-linux:
	./scripts/setup_linux.sh

setup-macos:
	./scripts/setup_macos.sh

run-demo:
	./scripts/run_demo.sh

run-failure-demo:
	./scripts/run_failure_demo.sh

run-live-3d:
	./scripts/run_live_sitl_gazebo_fox_hunt.sh

test:
	$(PYTHON) -m pytest src/tests -q
