// Headless interaction test: drags a mug across the table and checks that the planner sees it.
// Usage: node web/tools/drag_test.mjs http://localhost:8765
import { chromium } from 'playwright';

const [url = 'http://localhost:8765'] = process.argv.slice(2);
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
await page.goto(url);
await page.waitForFunction(() => window.motionAmigoDemo, null, { timeout: 30000 });
await page.evaluate(() => window.motionAmigoDemo.loadScene('tabletop'));
const before = await page.evaluate(() => window.motionAmigoDemo.objectPosition('mug_1'));
const [x, y] = await page.evaluate(() => window.motionAmigoDemo.screenPosition('mug_1'));
await page.mouse.move(x, y);
await page.mouse.down();
for (let i = 1; i <= 10; i++) await page.mouse.move(x + 12 * i, y - 4 * i);
await page.mouse.up();
const after = await page.evaluate(() => window.motionAmigoDemo.objectPosition('mug_1'));
const moved = Math.hypot(after[0] - before[0], after[1] - before[1]);
const sceneJson = await page.evaluate(() => window.motionAmigoDemo.sceneJson());
const inWasm = JSON.parse(sceneJson).objects.find((o) => o.id === 'mug_1').center;
console.log('moved by', moved.toFixed(3), 'm; status:', await page.textContent('#status'));
await browser.close();
if (moved < 0.05 || Math.abs(inWasm[0] - after[0]) > 1e-6) {
  console.error('drag test failed');
  process.exit(1);
}
