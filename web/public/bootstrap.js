import { loadingScreen } from './loading-screen.js?v=3';
import './viewport.js?v=safari-1';
import './browser-scroll.js?v=native-touch-1';

try {
  await import('./app.js?v=health-room-1');
  // Allow the first prepared map frame to reach the screen before fading out.
  await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
  loadingScreen.finish();
} catch (error) {
  loadingScreen.fail(error);
}
