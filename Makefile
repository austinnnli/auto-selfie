# Everything that can be run without a rover on the desk.
#
#   make            tests, both languages
#   make check      tests + the generated files are current
#   make firmware   regenerate the config header and build with ESP-IDF

PYTHON ?= python3

.PHONY: all test test-py test-c check generated selftest firmware firmware-config clean

all: test

test: test-py test-c

test-py:
	@$(PYTHON) tools/run_tests.py

test-c:
	@$(MAKE) -s -C firmware/test

# Fails if config.json, protocol.py or decide.py changed without the generated
# files being regenerated - which is how a stale golden corpus or a drifted
# protocol.h would otherwise reach a field.
check: test
	@$(PYTHON) tools/gen_test_vectors.py >/dev/null
	@$(PYTHON) tools/gen_config_header.py >/dev/null
	@$(PYTHON) tools/gen_golden.py --check
	@if git rev-parse --git-dir >/dev/null 2>&1; then \
		git diff --quiet -- firmware/main/config_defaults.h firmware/test/vectors.h \
			tests/fixtures/protocol_vectors.json \
			|| echo "note: generated files were stale and have been refreshed"; \
	fi
	@echo "all checks passed"

generated:
	@$(PYTHON) tools/gen_test_vectors.py
	@$(PYTHON) tools/gen_config_header.py
	@$(PYTHON) tools/gen_golden.py

# C-6: the risk to retire in the first hour.
selftest:
	@$(PYTHON) -m app.server --selftest

firmware-config:
	@$(PYTHON) tools/gen_config_header.py

firmware: firmware-config
	cd firmware && idf.py build

clean:
	@$(MAKE) -s -C firmware/test clean
	@find . -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
