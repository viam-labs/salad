package main

import (
	"context"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

func runFrames(address, apiKey, apiKeyID string) error {
	// TODO: Remove this one.
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("--address, --api-key, and --api-key-id are required")
	}

	ctx := context.Background()
	logger := logging.NewLogger("frames")

	robotClient, err := client.New(ctx, address, logger,
		client.WithDialOptions(
			rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			}),
		),
	)
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}
	defer robotClient.Close(ctx)

	cfg, err := robotClient.FrameSystemConfig(ctx)
	if err != nil {
		return fmt.Errorf("failed to get frame system config: %w", err)
	}

	fmt.Println(cfg.String())
	return nil
}
