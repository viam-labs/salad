
GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/salad

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/salad.exe
endif

node_modules: package.json
	npm ci

dist/index.html: node_modules src/*
	npm run build

build: $(MODULE_BINARY)

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go dist/index.html
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/main.go

bin/salad-cli: go.mod cmd/cli/*.go segmentation/*.go meshifier/main.py meshifier/algos.py
	go build -o bin/salad-cli ./cmd/cli

cli: bin/salad-cli

# Yes this regex could be more specific but making it more specific in a way
# that works the same across GNU and BSD grep isn't currently worth the effort.
GOVERSION = $(shell grep '^go .\..' go.mod | head -n1 | cut -d' ' -f2)

# Optional: specify files to lint (e.g., make lint LINT_FILES="file1.go file2.go")
LINT_FILES ?=

GOLANGCI_LINT_VERSION = v1.62.2

lint-fix:
	GOTOOLCHAIN=go$(GOVERSION) GOGC=50 go run github.com/golangci/golangci-lint/cmd/golangci-lint@$(GOLANGCI_LINT_VERSION) run -v --fix --config=./etc/golangci.yaml $(LINT_FILES)

lint: lint-fix
	go fix ./...
	go mod tidy

# Reports lint issues without fixing them
check-lint:
	go mod tidy
	GOTOOLCHAIN=go$(GOVERSION) GOGC=50 go run github.com/golangci/golangci-lint/cmd/golangci-lint@$(GOLANGCI_LINT_VERSION) run -v --config=./etc/golangci.yaml $(LINT_FILES)

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY) dist/index.html meshifier/main.py meshifier/algos.py meshifier/requirements.txt
ifneq ($(VIAM_TARGET_OS), windows)
	strip $(MODULE_BINARY)
endif
	tar czf $@ meta.json $(MODULE_BINARY) dist meshifier/main.py meshifier/algos.py meshifier/requirements.txt

module: test module.tar.gz

all: test module.tar.gz

setup:
	bash ./first_run.sh
	go mod tidy
	which npm > /dev/null 2>&1 || (curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - && apt-get -y install nodejs)

.PHONY: build lint lint-fix check-lint va-update va-upload

va-update: meta.json
	viam module update --module=meta.json

# Viam Application bundle, per https://docs.viam.com/build-apps/hosting/deploy/.
# Named viam-app.tar.gz to avoid colliding with the per-platform module.tar.gz.
viam-app.tar.gz: meta.json dist/index.html
	tar -czvf $@ meta.json dist/

va-upload: viam-app.tar.gz
	@test -n "$(VERSION)" || (echo "VERSION required, e.g. make va-upload VERSION=0.0.108"; exit 1)
	# --force: the CLI validates the module entrypoint (bin/salad) is in the
	# archive, but the platform=any upload is the application bundle, not the
	# module binary. Likely a Viam CLI bug for hybrid modules.
	viam module upload --force --upload viam-app.tar.gz --platform any --version $(VERSION)
