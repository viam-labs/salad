# Salad Ordering App

Sweetgreen-inspired web UI for building and ordering salads from the `salad-coordinator` service. Built with Svelte 5 and the Viam TypeScript SDK.

## Setup

```bash
npm install
npm run dev
```

For local development, hardcoded credentials in `src/lib/robot.ts` connect to the robot. When deployed as a Viam module app, credentials are read from the platform cookie automatically.

## Build

```bash
npm run build
```

Outputs to `dist/` which is served as the module app entrypoint.

## Architecture

The app has three screens:

1. **Ordering** — Fetches ingredients from `salad-coordinator` via `list_ingredients`, displays them in a categorized grid with portion steppers and a floating cart footer.
2. **Building** — Sends `build_salad` to the coordinator, polls `status` every second, and shows a live progress bar.
3. **Complete** — Confirmation screen that auto-returns to ordering after 10 seconds.

## Stack

- **Svelte 5** (runes: `$state`, `$derived`, `$props`) + **Vite**
- **@viamrobotics/sdk** — `GenericServiceClient.doCommand()` for all robot communication
- **No framework router or SSR** — static SPA, appropriate for Viam module apps

## File Structure

```
src/
  main.ts                          # Mounts <App /> into #app
  App.svelte                       # Connection init, screen routing
  styles.css                       # Global CSS (Sweetgreen palette)
  lib/
    robot.ts                       # Viam SDK connection + doCommand wrappers
    types.ts                       # Ingredient, AppScreen types
    constants.ts                   # Emoji map, portion limits, categories
  components/
    OrderingScreen.svelte          # Ingredient grid + cart footer
    CategorySection.svelte         # Category heading + tile grid
    IngredientTile.svelte          # Emoji, name, +/- stepper
    BuildingScreen.svelte          # Progress bar + status polling
    CompleteScreen.svelte          # Success message + countdown
```

## Design

- **Colors:** `#F4F3E7` (hummus), `#d8e5d6` (cucumber), `#e8dcc6` (quinoa)
- **Fonts:** Grenette (serif display) for headings, DM Sans for body
- **Tiles:** `border-radius: 1.25rem`, white background, subtle shadow
