# Physical AI & Humanoid Robotics: Agent-Native Future

This repository contains a comprehensive textbook on Physical AI and Humanoid Robotics, built with Docusaurus.

## Structure

The book is organized into 10 modules:

1. **Module 1** – Robotic Nervous System (ROS 2)
2. **Module 2** – Digital Twin (Gazebo & Unity)
3. **Module 3** – Perception System (Computer Vision + Sensors)
4. **Module 4** – Humanoid Movement & Control
5. **Module 5** – Humanoid AI Brain (Cognition + Decision Making)
6. **Module 6** – Multi-Agent Intelligence System
7. **Module 7** – Memory & Long-Term Learning
8. **Module 8** – Reinforcement Learning & Training
9. **Module 9** – Human–Robot Interaction (HRI)
10. **Module 10** – Physical AI Capstone Project

## Deployment Options

### Option 1: GitHub Pages (Automatic)

The site is configured for deployment to GitHub Pages. To deploy:

1. Push your changes to the `main` branch
2. GitHub Actions will automatically build and deploy your site
3. The site will be available at: `https://your-username.github.io/ai-native-book`

### Option 2: Vercel (Recommended)

To deploy to Vercel:

1. Push this repository to GitHub
2. Go to [Vercel](https://vercel.com)
3. Click "New Project"
4. Import your repository
5. Use the following configuration:
   - Framework: Docusaurus
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Root Directory: `.` (root)
6. Click "Deploy"

### Option 3: Netlify

To deploy to Netlify:

1. Push this repository to GitHub
2. Go to [Netlify](https://netlify.com)
3. Click "Add new site"
4. Select your repository
5. For build settings, use:
   - Build command: `npm run build`
   - Publish directory: `build`
6. Click "Deploy site"

## Local Development

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start the development server:
   ```bash
   npm start
   ```

This command starts a local development server and opens a browser window. Most changes are reflected live without having to restart the server.

## Build for Production

To build the static files for production:

```bash
npm run build
```

## Contributing

We welcome contributions to improve the textbook content. Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-topic`)
3. Commit your changes (`git commit -m 'Add amazing topic'`)
4. Push to the branch (`git push origin feature/amazing-topic`)
5. Open a pull request

## License

This textbook is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.