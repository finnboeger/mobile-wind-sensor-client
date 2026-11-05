# Local dashboard

Vue app served by `src/dashboard/server.py`. `npm run build` type-checks it and writes
the bundle to `../static`, which is gitignored. CI builds it into every release.

- `npm ci && npm run build` builds it, e.g. on a Pi installed with `setup.sh --git`.
  On a Pi Zero 2 W, `npm run build:fast` skips the type check, which needs a lot of
  memory. Building on an original Pi Zero or Zero W (ARMv6) isn't supported, since
  there are no Node.js or Vite builds for it: install a release there.
- `WINDBOT_URL=http://10.42.0.1 npm run dev` serves it with hot reload and forwards
  `/api` to a running windbot.
