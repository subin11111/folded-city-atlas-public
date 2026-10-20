import { loadingScreen } from './loading-screen.js?v=3';

try {
  await import('./app.js?v=walking-guide-1');
  // Allow the first prepared map frame to reach the screen before fading out.
  await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
  loadingScreen.finish();
} catch (error) {
  loadingScreen.fail(error);
}
