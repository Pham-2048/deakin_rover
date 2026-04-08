# Deakin Rover GUI

Ground control station interface for the Deakin Rover. Connects to the rover over WebSocket and provides real-time telemetry, camera feeds, and remote control.

## Tech Stack

- [Next.js 16](https://nextjs.org) (App Router)
- [React 19](https://react.dev)
- [MUI v7](https://mui.com) (Material UI)
- [roslib 2.0](https://github.com/RobotWebTools/roslibjs) (WebSocket bridge to ROS 2)

## Quick Start

```bash
npm install
npm run dev
```

Open http://localhost:3000 and enter the rover's IP address to connect.

## Project Structure

```
src/
  app/            # Next.js pages and layout
  components/     # UI components
    camera/       # Camera feed grid and controls
    control/      # Node manager, e-stop
    layout/       # App shell, sidebar, connection manager
    status/       # System status dashboard
    common/       # Shared components
  context/        # React Context providers (ROS connection, system status)
  hooks/          # Custom React hooks
  lib/            # ROS bridge singleton, constants, utilities
  styles/         # Theme configuration
```

## Scripts

| Command | Description |
|---------|-------------|
| `npm run dev` | Start dev server on port 3000 |
| `npm run build` | Production build |
| `npm run start` | Serve production build |
| `npm run lint` | Run ESLint |

## License

See the repository root for license information.
