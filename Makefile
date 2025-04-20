
.PHONY: build
build:
	@echo "Building project..."
	@cmake --preset=default
	@cmake --build build

.PHONY:test
test: build
	@echo "Running tests..."
	@cd build && ctest --output-on-failure

.PHONY: coverage
coverage: coverage-reset test
	@echo "Running coverage..."
	@mkdir -p coverage
	@lcov -c -d . --no-external --output-file coverage.info 
	@genhtml coverage.info --output-directory coverage


.PHONY: coverage-reset
coverage-reset:
	@echo "Resetting coverage..."
	@lcov --directory . --zerocounters

.PHONY: clean
clean:
	@echo "Cleaning build directory..."
	@rm -rf build/CMakeFiles
	@rm -rf build/coverage
	@rm -rf build/CMakeCache.txt
	@rm -rf build/Testing
	@rm -rf build/routing