#!/usr/bin/env node
// Optional real-browser verification. Install Playwright outside the runtime image.
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {readFile} from 'node:fs/promises';
const {chromium} = createRequire(import.meta.url)(process.env.PLAYWRIGHT_MODULE || 'playwright');
const args = process.argv.slice(2);
const option = name => {const i=args.indexOf(name);return i<0?undefined:args[i+1];};
const browser = await chromium.launch({headless:true});
try {
  const page = await browser.newPage({viewport:{width:1440,height:1360},deviceScaleFactor:1});
  const errors=[];
  page.on('pageerror',error=>errors.push(error.message));
  // MJPEG connections intentionally stay open in a live experiment.
  await page.goto(option('--url') || 'http://localhost:8770', {waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.getElementById('connection').textContent.includes('connected'));
  await page.waitForFunction(()=>document.getElementById('radiation-status').textContent.includes('GP update'));
  await page.waitForFunction(()=>typeof radiation!=='undefined'&&radiation.predicted_cells>0);
  await page.waitForFunction(()=>['blind','aware'].every(mode=>document.getElementById(mode+'-frame').naturalWidth===640));
  assert.equal(await page.locator('#milestone-table tr').count(),4);
  for(const mode of ['blind','aware']) {
    assert.equal(await page.locator(`#${mode}-frame`).evaluate(image=>image.naturalWidth),640);
  }
  if(option('--report')) {
    const report=JSON.parse(await readFile(option('--report'),'utf8'));
    const milestone=report.matched.coverage_milestones.find(row=>row.reference_percent===90);
    assert.equal(await page.locator('#progress-effect').textContent(),Math.abs(milestone.aware_dose_reduction_percent).toFixed(1)+'%');
    assert.equal((await page.locator('#completion-value').textContent()).replaceAll(' ',''),'0/2');
    for(const mode of ['blind','aware'])assert.match(await page.locator(`#${mode}-badge`).textContent(),/incomplete/);
  }
  // Check that both rate canvases have rendered, rather than just existing in DOM.
  for(const id of ['truth-map','gp-map']) {
    assert(await page.locator('#'+id).evaluate(canvas=>{
      const data=canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data;
      const colors=new Set();for(let i=0;i<data.length;i+=40)colors.add(`${data[i]},${data[i+1]},${data[i+2]}`);
      return colors.size>8;
    }));
  }
  if(option('--screenshot'))await page.screenshot({path:option('--screenshot')});
  await page.setViewportSize({width:390,height:844});
  await page.waitForTimeout(150);
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth),'Mobile layout overflows the viewport');
  await page.locator('#benchmark-level').selectOption('75');
  await page.waitForFunction(()=>document.getElementById('progress-label').textContent.includes('75%'));
  assert.deepEqual(errors,[]);
  console.log('Browser checks passed: paired metrics, endpoint labels, videos, rate maps, milestone selector and mobile layout.');
} finally {
  await browser.close();
}
