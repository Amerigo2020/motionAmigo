// Records the README animation headlessly: plans in the shelf scene and steps through the
// trajectories frame by frame (deterministic, independent of the renderer's speed).
//
// Usage: node web/tools/record.mjs http://localhost:8765 frames-dir
// Then:  uv run --with pillow python web/tools/make_gif.py frames-dir docs/media/demo.gif
import { mkdirSync } from 'node:fs';
import { chromium } from 'playwright';

const [url = 'http://localhost:8765', dir = 'frames'] = process.argv.slice(2);
mkdirSync(dir, { recursive: true });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1200, height: 675 } });
page.on('pageerror', (e) => console.log('[pageerror]', e.message));
await page.goto(url);
await page.waitForFunction(() => window.motionAmigoDemo, null, { timeout: 30000 });
await page.evaluate(() => window.motionAmigoDemo.loadScene('shelf'));

let frame = 0;
const shot = async (repeat = 1) => {
  await page.evaluate(() => window.motionAmigoDemo.renderNow());
  for (let r = 0; r < repeat; r++) {
    await page.screenshot({ path: `${dir}/frame_${String(frame++).padStart(4, '0')}.png` });
  }
};

await shot(6);
for (const goal of ['middle left', 'upper', 'lower', 'middle right']) {
  await page.evaluate((g) => window.motionAmigoDemo.selectGoalByName(g), goal);
  await shot(3);
  const n = await page.evaluate(() => window.motionAmigoDemo.planStatic());
  const frames = 22;
  for (let k = 0; k <= frames; k++) {
    const i = Math.round((k / frames) * (n - 1));
    await page.evaluate((j) => window.motionAmigoDemo.showSample(j), i);
    await shot();
  }
  await shot(6);
  console.log(goal, await page.textContent('#status'));
}
await browser.close();
console.log(`${frame} frames written to ${dir}`);
