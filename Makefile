
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

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go dist/index.html
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/main.go

bin/salad-cli: go.mod cmd/cli/*.go meshifier/main.py meshifier/algos.py
	go build -o bin/salad-cli ./cmd/cli

cli: bin/salad-cli

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY) dist/index.html
ifneq ($(VIAM_TARGET_OS), windows)
	strip $(MODULE_BINARY)
endif
	tar czf $@ meta.json $(MODULE_BINARY) dist

module: test module.tar.gz

all: test module.tar.gz

setup:
	go mod tidy
	which npm > /dev/null 2>&1 || (curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - && apt-get -y install nodejs)

.PHONY: va-update va-upload

va-update: meta.json
	viam module update --module=meta.json

VA_VERSION ?= 0.0.16

va-upload:
	viam module upload --version=${VA_VERSION} --platform=any --public-namespace=ncs .
