# Physical AI & Humanoid Robotics Documentation

This website is built using [Docusaurus 3](https://docusaurus.io/), a modern static website generator.

### Installation

```bash
npm install
```

### Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Deployment

The site is automatically deployed to GitHub Pages via the GitHub Actions workflow in `.github/workflows/deploy.yml` when changes are pushed to the `main` branch.

### Project Structure

- `docusaurus.config.ts`: Main configuration file
- `sidebars.ts`: Navigation sidebar configuration
- `src/`: Custom React components and pages
- `docs/`: Documentation content organized by modules
- `static/`: Static assets like images and icons