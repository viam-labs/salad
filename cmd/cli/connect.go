package main

import (
	"context"
	"fmt"
	"os"
	"strings"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

// MachineDialFlags bundles the connection flags consumed by every command
// that talks to a live robot. We pull values from the global persistent
// flags (--address, --api-key, --api-key-id), but also honor the
// VIAM_ADDRESS / VIAM_API_KEY / VIAM_API_KEY_ID env vars as fallbacks so
// users don't have to repeat them on every invocation.
type MachineDialFlags struct {
	Address  string
	APIKey   string
	APIKeyID string
}

func resolveDialFlags() (MachineDialFlags, error) {
	f := MachineDialFlags{
		Address:  strings.TrimSpace(firstNonEmpty(globalAddress, os.Getenv("VIAM_ADDRESS"))),
		APIKey:   strings.TrimSpace(firstNonEmpty(globalAPIKey, os.Getenv("VIAM_API_KEY"))),
		APIKeyID: strings.TrimSpace(firstNonEmpty(globalAPIKeyID, os.Getenv("VIAM_API_KEY_ID"))),
	}
	if f.Address == "" {
		return f, fmt.Errorf("--address is required (or set VIAM_ADDRESS)")
	}
	if f.APIKey == "" || f.APIKeyID == "" {
		return f, fmt.Errorf("--api-key and --api-key-id are required (or set VIAM_API_KEY / VIAM_API_KEY_ID)")
	}
	return f, nil
}

func firstNonEmpty(vals ...string) string {
	for _, v := range vals {
		if v != "" {
			return v
		}
	}
	return ""
}

// dialMachine opens a robot client connection to a Viam machine using API
// key credentials. Callers MUST defer Close on the returned client.
func dialMachine(ctx context.Context, logger logging.Logger) (*client.RobotClient, error) {
	flags, err := resolveDialFlags()
	if err != nil {
		return nil, err
	}
	logger.Infof("Dialing %s", flags.Address)
	rc, err := client.New(
		ctx,
		flags.Address,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			flags.APIKeyID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: flags.APIKey,
			},
		)),
	)
	if err != nil {
		return nil, fmt.Errorf("connecting to %q: %w", flags.Address, err)
	}
	return rc, nil
}
