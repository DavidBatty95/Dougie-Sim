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
  assert.equal(await page.title(),'Dougie-Sim');
  assert.equal(await page.locator('h1').innerText(),'Dougie-Sim');
  assert.equal(await page.locator('#dose-progress').count(),0);
  await page.waitForFunction(()=>document.getElementById('radiation-status').textContent.includes('GP update'));
  if(!args.includes('--campaign-death'))await page.waitForFunction(()=>typeof radiation!=='undefined'&&radiation.predicted_cells>0);
  await page.waitForFunction(()=>['blind','aware'].every(mode=>document.getElementById(mode+'-frame').naturalWidth===640));
  assert.equal(await page.locator('#milestone-table tr').count(),4);
  for(const mode of ['blind','aware']) {
    const image=page.locator(`#${mode}-frame`);
    assert.equal(await image.evaluate(image=>image.naturalWidth),640);
    assert(await image.isVisible(),`${mode} feed received a frame but is hidden`);
    // MJPEG onload may wait for the stream to end. Verify actual decoded image
    // updates instead of treating naturalWidth as proof of a visible live feed.
    if(await page.evaluate(mode=>latest[mode].exit_code===null&&latest[mode].rows.at(-1)?.robot_lifetime?.alive!==false,mode)){
      const hash=await image.evaluate(image=>{
        const c=document.createElement('canvas');c.width=64;c.height=48;
        const ctx=c.getContext('2d');ctx.drawImage(image,0,0,64,48);
        return Array.from(ctx.getImageData(0,0,64,48).data).reduce((hash,v)=>Math.imul(hash^v,16777619),2166136261)>>>0;
      });
      await page.waitForFunction(({mode,hash})=>{
        if(latest[mode].exit_code!==null)return true;
        const image=document.getElementById(mode+'-frame'),c=document.createElement('canvas');c.width=64;c.height=48;
        const ctx=c.getContext('2d');ctx.drawImage(image,0,0,64,48);
        return (Array.from(ctx.getImageData(0,0,64,48).data).reduce((value,v)=>Math.imul(value^v,16777619),2166136261)>>>0)!==hash;
      },{mode,hash},{timeout:15000});
    }
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
    if(id==='gp-map'&&args.includes('--campaign-death')&&await page.evaluate(()=>radiation.predicted_cells===0))continue;
    assert(await page.locator('#'+id).evaluate(canvas=>{
      const data=canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data;
      const colors=new Set();for(let i=0;i<data.length;i+=40)colors.add(`${data[i]},${data[i+1]},${data[i+2]}`);
      return colors.size>8;
    }));
  }
  if(option('--screenshot'))await page.screenshot({path:option('--screenshot')});
  if(args.includes('--campaign-death')){
    await page.waitForFunction(()=>sessionState?.campaign?.status==='halted'&&modes.every(m=>latest[m].exit_code!==null));
    const data=await page.evaluate(()=>({latest,campaign:sessionState.campaign}));
    for(const mode of ['blind','aware']){
      const run=data.latest[mode],row=run.rows.at(-1);
      assert.equal(row.state,'dead_radiation');assert.equal(row.robot_lifetime.alive,false);
      assert.equal(run.summary.simulation.powered_off,true);
      assert(row.true_dose_usv>=row.robot_lifetime.limit_usv);
      assert.equal(data.campaign.modes[mode].robots_lost,1);
      assert.match(await page.locator('#'+mode+'-badge').textContent(),/Destroyed.*in situ/);
      assert(await page.locator('#'+mode+'-life-panel').isVisible());
    }
    assert.match(await page.locator('#campaign-status').textContent(),/origin.*blocked/);
    assert(await page.locator('#rerun').isDisabled());
    assert(await page.locator('#learn-planning').isDisabled());
    console.log('Campaign failure passed: actual radiation deaths, frozen final poses, visible lifetime/death labels, retained wrecks and origin-blocked deployment guard.');
    if(args.includes('--campaign-reset')){
      await page.locator('#campaign-limit').fill('100');
      await page.locator('#campaign-sorties').fill('2');
      await page.locator('#new-campaign').click();
      await page.waitForFunction(id=>sessionState.campaign.id>id&&sessionState.campaign.limit_usv===100&&modes.every(m=>latest[m].rows.at(-1)?.robot_lifetime?.robot_id===1&&latest[m].rows.at(-1)?.robot_lifetime?.alive),data.campaign.id,{timeout:60000});
      const fresh=await page.evaluate(()=>sessionState.campaign);
      assert.equal(fresh.modes.blind.wrecks,0);assert.equal(fresh.modes.aware.wrecks,0);
      assert.equal(fresh.sorties_finished,0);
      console.log('New-campaign control passed: explicit new experiment, fresh robots and cleared wrecks while previous results remain retained.');
    }
  }
  if(args.includes('--campaign-recovery')){
    await page.waitForFunction(()=>sessionState?.campaign?.sorties_finished>=2,undefined,{timeout:90000});
    const data=await page.evaluate(()=>({latest,campaign:sessionState.campaign}));
    assert.equal(data.campaign.status,'sortie_limit');
    for(const mode of ['blind','aware']){
      const row=data.latest[mode].rows[0],previous=data.campaign.history[0].results[mode];
      assert.equal(data.campaign.modes[mode].robots_recovered,2);
      assert.equal(data.campaign.modes[mode].robots_lost,0);
      assert.equal(row.robot_lifetime.robot_id,2);
      assert(row.true_dose_usv<previous.robot_lifetime.absorbed_usv);
      assert(Math.abs(row.campaign_dose_usv-previous.total_dose_usv-row.true_dose_usv)<1e-10);
      assert(row.explored_percent>=previous.explored_percent);
      assert(data.latest[mode].summary.campaign.prior_directory);
    }
    assert(data.campaign.history[1].results.aware.gp_updates>data.campaign.history[0].results.aware.gp_updates);
    assert(await page.locator('#campaign').isVisible());
    assert.match(await page.locator('#aware-badge').textContent(),/Recovered/);
    console.log('Campaign hand-off passed: two real sorties, recovered robots, fresh lifetime dose, retained GP/map/progress and cumulative exposure accounting.');
  }
  if(args.includes('--continuous-field')) {
    await page.waitForFunction(()=>radiation.interpolation==='compact_wendland_shepard');
    const field=await page.evaluate(()=>radiation);
    assert(field.exported_gp_queries>0&&field.exported_gp_queries<=8192);
    assert(field.gp_neighbours.some(n=>n>1));
    for(let i=0;i<field.gp_mean_usv_h.length;i++)if(field.gp_mean_usv_h[i]!==null){
      assert(field.gp_std_log_rate[i]>=0);
      assert(field.gp_upper_usv_h[i]>=field.gp_mean_usv_h[i]-1e-9);
    }
    assert.match(await page.locator('#field-method').textContent(),/Continuous compact-kernel interpolation/);
    console.log(`Continuous field passed: ${field.exported_gp_queries} direct GP queries, multi-neighbour interpolation, uncertainty and explicit display semantics.`);
  }
  if(args.includes('--radiation-exclusion')) {
    await page.waitForFunction(()=>latest.aware.exit_code!==null);
    const row=await page.evaluate(()=>latest.aware.rows.at(-1));
    assert.equal(row.survey_outcome,'radiation_limited');
    assert.equal(row.frontier_complete,false);
    assert(row.frontier_count>0);
    assert.match(await page.locator('#aware-badge').textContent(),/Radiation-limited.*incomplete/);
    assert.match(await page.locator('#aware-blocked').textContent(),/model confidence/);
    assert.equal(await page.evaluate(()=>latest.blind.rows.at(-1).radiation_exclusion),null);
    console.log('Radiation exclusions passed: actual aware worker returned radiation-limited with unresolved frontiers; blind exclusions disabled.');
  }
  if(args.includes('--controls')) {
    await page.waitForFunction(()=>sessionState&&!document.getElementById('rerun').disabled);
    const first=await page.evaluate(()=>sessionState.run_id);
    await page.locator('#continuous').check();
    await page.waitForFunction(()=>sessionState.continuous);
    await page.waitForFunction(id=>sessionState.run_id>id,first,{timeout:90000});
    await page.locator('#continuous').uncheck();
    await page.waitForFunction(()=>!sessionState.continuous&&!document.getElementById('rerun').disabled);
    const second=await page.evaluate(()=>sessionState.run_id);
    await page.locator('#rerun').click();
    await page.waitForFunction(id=>sessionState.run_id>id,second,{timeout:60000});
    await page.waitForFunction(()=>averageData?.paired_observations>=1);
    assert((await page.locator('#run-history tr').count())>=3);
    for(const id of ['average-dose','average-distance']) {
      const canvas=page.locator('#'+id);
      assert(await canvas.evaluate(c=>{
        const pixels=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
        for(let i=0;i<pixels.length;i+=4)if((pixels[i]===255&&pixels[i+1]===180)||(pixels[i]===72&&pixels[i+1]===218))return true;
        return false;
      }),'Mean graph did not render either trajectory');
      await canvas.hover();
      await page.waitForFunction(key=>document.getElementById(key+'-detail').textContent.includes('n ='),id);
    }
    console.log('Run controls passed: continuous next-seed restart, manual rerun, retained history, rendered means and contributor counts.');
  }
  if(args.includes('--learning')){
    await page.waitForFunction(()=>sessionState?.learning_enabled&&learningData?.learning?.entries.length>=2,undefined,{timeout:90000});
    assert(await page.locator('#learn-planning').isChecked());
    const data=await page.evaluate(()=>learningData);
    const [incumbent,challenger]=data.learning.entries;
    assert.equal(incumbent.trial.role,'incumbent');
    assert.equal(challenger.trial.role,'challenger');
    assert.equal(incumbent.trial.seed,challenger.trial.seed);
    assert.notDeepEqual(incumbent.trial.weights,challenger.trial.weights);
    for(const record of data.runs)assert.deepEqual(record.planning_weights.blind,{route_dose_weight:1,frontier_dose_weight:2});
    assert((await page.locator('#learning-history tr').count())>=2);
    assert.match(await page.locator('#learning-rule').textContent(),/total dose on every seed/);
    assert(data.runs.some(r=>r.learning_result?.eligible),'Short functional trials produced no comparable-progress points');
    for(const id of ['learning-dose','learning-distance']){
      assert(await page.locator('#'+id).evaluate(c=>{
        const pixels=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
        for(let i=0;i<pixels.length;i+=4)if((pixels[i]===255&&pixels[i+1]===180)||(pixels[i]===72&&pixels[i+1]===218))return true;
        return false;
      }),'Learning graph did not render an actual trial point');
    }
    await page.locator('#learn-planning').uncheck();
    await page.waitForFunction(()=>!sessionState.learning_enabled&&!document.getElementById('continuous').disabled);
    await page.locator('#continuous').uncheck();
    await page.waitForFunction(()=>!sessionState.continuous);
    console.log('Learning controls passed: fixed blind baseline, same-seed incumbent/challenger trials, visible dose/distance progression and pause controls.');
  }
  await page.setViewportSize({width:390,height:844});
  await page.waitForTimeout(150);
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth),'Mobile layout overflows the viewport');
  await page.locator('#benchmark-level').selectOption('75');
  await page.waitForFunction(()=>document.getElementById('progress-label').textContent.includes('75%'));
  if(await page.evaluate(()=>Boolean(sessionState))) {
    // Recreate a tab holding an ended session while the server starts another
    // world with the same numeric run ID and a lower revision counter.
    await page.evaluate(()=>{
      sessionState={...sessionState,output:'/previous-dashboard-session',revision:999999};
      currentRun=sessionState.output;historyRevision=999999;
      for(const mode of modes){const image=document.getElementById(mode+'-frame');image.dataset.finished='true';image.src='/'+mode+'/frame.jpg?old-session';}
    });
    await page.waitForFunction(()=>currentRun===latest._session.output&&sessionState.revision<999999&&historyRevision>=0);
    for(const mode of ['blind','aware']){
      const image=page.locator('#'+mode+'-frame');
      assert(await image.isVisible(),`${mode} feed hidden after changing sessions`);
      if(await page.evaluate(mode=>latest[mode].exit_code===null,mode)){
        assert.equal(await image.getAttribute('data-finished'),null);
        assert.match(await image.getAttribute('src'),/stream\.mjpg/);
      }
    }
  }
  assert.deepEqual(errors,[]);
  console.log('Browser checks passed: visible/updating feeds, session-switch reconnection, paired metrics, endpoint labels, rate maps, milestone selector and mobile layout.');
} finally {
  await browser.close();
}
