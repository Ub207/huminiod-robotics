# Deployment Guide

## Deploy to Vercel

To deploy your Docusaurus site to Vercel:

### Method 1: One-Click Deploy
[![Deploy to Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/your-username/ai-native-book&project-name=ai-native-book&repo-name=ai-native-book)

### Method 2: Manual Deployment

1. Install Vercel CLI (optional):
```bash
npm i -g vercel
```

2. If using the CLI, link your project:
```bash
vercel
```

3. Deploy:
```bash
vercel --prod
```

### Method 3: Git Integration (Recommended)
1. Push your repository to GitHub/GitLab/Bitbucket
2. Go to [Vercel Dashboard](https://vercel.com/dashboard)
3. Click "Add New Project"
4. Import your repository
5. In the configuration page:
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Development Command: `docusaurus start`
6. Click "Deploy"

## Environment Configuration for Vercel

The site is already configured for Vercel deployment through the `vercel.json` file in the root directory:

```json
{
  "framework": "docusaurus",
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "build"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ]
}
```

## Vercel Settings

Recommended project settings in Vercel dashboard:
- Framework Preset: Docusaurus
- Build Command: `npm run build`
- Output Directory: `build`
- Install Command: `npm install`

## Preview Deployments

When you push to any branch, Vercel automatically creates a preview deployment. Pull requests will also have deployment previews.

## Custom Domain

To add a custom domain:
1. Go to your project in the Vercel dashboard
2. Go to Settings > Domain
3. Add your custom domain
4. Follow the instructions to configure DNS settings