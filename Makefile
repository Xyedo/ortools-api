
.PHONY: build
build:
	@echo "Building project..."
	@cmake --build build

.PHONY:test
test: build
	@echo "Running tests..."
	@cd build && ctest --output-on-failure
