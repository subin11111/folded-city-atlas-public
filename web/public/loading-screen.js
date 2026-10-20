const screen = document.getElementById('loadingScreen');
const message = document.getElementById('loadingMessage');
const retry = document.getElementById('loadingRetry');
const shell = [...document.querySelectorAll('.topbar, .workspace, .status-strip')];
const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
const started = performance.now();
for (const element of shell) {
  element.inert = true;
  element.setAttribute('aria-busy', 'true');
}
let phase = 'loading';
let finishTimer;
const slowTimer = setTimeout(() => {
  if (phase !== 'loading') return;
  message.textContent = '지도를 준비하는 데 시간이 조금 걸리고 있어요.';
  retry.hidden = false;
}, 12000);
retry.addEventListener('click', () => window.location.reload());

// The initial SVG is in the HTML so the animation appears before map modules arrive.
const compact = document.getElementById('modelLoading');
const artwork = document.getElementById('loadingArtwork').cloneNode(true);
artwork.removeAttribute('id');
const compactMessage = document.createElement('span');
compactMessage.className = 'model-loading-message';
compact.replaceChildren(artwork, compactMessage);
compact.hidden = true;

export const loadingScreen = {
  model(text) {
    compactMessage.textContent = text;
    compact.hidden = !text;
  },
  finish() {
    if (phase !== 'loading') return;
    phase = 'finishing';
    clearTimeout(slowTimer);
    // Let the fold and arrow complete once on fast connections.
    const remaining = reducedMotion.matches ? 0 : Math.max(0, 2500 - (performance.now() - started));
    finishTimer = setTimeout(() => {
      message.textContent = '공간이 연결되었습니다.';
      screen.classList.add('is-leaving');
      for (const element of shell) {
        element.inert = false;
        element.removeAttribute('aria-busy');
      }
      setTimeout(() => { screen.hidden = true; phase = 'ready'; }, reducedMotion.matches ? 0 : 400);
    }, remaining);
  },
  fail(error) {
    clearTimeout(slowTimer);
    clearTimeout(finishTimer);
    phase = 'error';
    screen.hidden = false;
    screen.classList.remove('is-leaving');
    screen.classList.add('is-error');
    screen.setAttribute('role', 'alert');
    screen.setAttribute('aria-label', '지도 불러오기 실패');
    message.textContent = '지도를 불러오지 못했어요. 연결을 확인하고 다시 시도해 주세요.';
    retry.hidden = false;
    console.error('Folded City startup:', error);
  },
};
