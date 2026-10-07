// Takes a screenshot of the demo (used for quick visual checks).
// Usage: node web/tools/screenshot.mjs http://localhost:8765 out.png [scene] [goal]
import { chromium } from 'playwright';

const [url = 'http://localhost:8765', out = 'demo.png', scene = 'tabletop', goal] = process.argv.slice(2);
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
page.on('console', (m) => console.log('[browser]', m.type(), m.text()));
page.on('pageerror', (e) => console.log('[pageerror]', e.message));
await page.goto(url);
await page.waitForFunction(() => window.motionAmigoDemo, null, { timeout: 30000 });
await page.evaluate((s) => window.motionAmigoDemo.loadScene(s), scene);
if (goal) {
  await page.evaluate((g) => window.motionAmigoDemo.selectGoalByName(g), goal);
  await page.evaluate(() => window.motionAmigoDemo.plan());
  await page.waitForFunction(() => !window.motionAmigoDemo.isAnimating(), null, { timeout: 60000 });
}
await page.waitForTimeout(300);
await page.screenshot({ path: out });
console.log(await page.textContent('#status'));
await browser.close();
