import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
// sidebars.ts (example)
const sidebars = {
  tutorialSidebar: [
    "intro",
    "CONSTITUTION", // <-- use this exact id (case-sensitive)
    "preface",
    {
      type: "category",
      label: "Modules",
      items: [
        "modules/module1-ros2",
        // ...
      ],
    },
  ],
};
export default sidebars;
