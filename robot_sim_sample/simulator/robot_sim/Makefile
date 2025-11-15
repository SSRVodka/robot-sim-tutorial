
.DEFAULT_GOAL := build

.PHONY: build clean rebuild help

build:
	colcon build

clean:
	rm -rf build install log

rebuild: clean build

help:
	@echo "Usage: make [target]"
	@echo "  make [build]  - Build this project"
	@echo "  make rebuild  - Clean and build this project"
	@echo "  make clean    - Clean this project"
	@echo "  make help     - Print this help message"