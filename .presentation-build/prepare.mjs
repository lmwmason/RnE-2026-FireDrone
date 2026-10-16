import fs from "node:fs/promises";
import sharp from "sharp";
const a=new URL("./assets/",import.meta.url).pathname;
async function out(src,name,crop,trim=false){let im=sharp(src,{density:150}).flatten({background:"#ffffff"});if(crop) im=im.extract(crop);if(trim) im=im.trim({threshold:25});await im.png().toFile(a+name+".png");}
await out(a+"bec-pcb.png","board",{left:330,top:212,width:660,height:645});
await out(a+"bec-schematic.png","power",{left:330,top:394,width:1110,height:423});
await out(a+"guide-4.png","rig",{left:854,top:0,width:746,height:900});
await out(a+"x8-dashboard.png","dashboard");
await out(a+"x8-dashboard.png","cover-x8",{left:10,top:195,width:710,height:435});
await out(a+"arm-now.png","arm");
await out("simulation/simulator/out/viz_prop_damage.png","prop");
await out("simulation/simulator/out/patched_ukf_gust.png","gust");
await out("simulation/simulator/out/patched_ukf_gauntlet.png","gauntlet");
await out(a+"simulink.png","simulink-bw",{left:0,top:105,width:500,height:296});
await out(a+"fc/FC-PCB.svg","fc",null,true);
await out(a+"fc/FC-PCB-MCU_H7_AI.svg","h7",null,true);
await out(a+"fc/FC-PCB-MCU_F4_Sensor.svg","f4",null,true);
console.log("Assets prepared");

await out("simulation/simulator/out/viz_prop_damage.png","prop-scene",{left:0,top:50,width:950,height:830});
await out("simulation/simulator/out/patched_ukf_gust.png","ukf-chart",{left:0,top:670,width:553,height:318});
await out("simulation/simulator/out/patched_ukf_gust.png","adrc-chart",{left:558,top:670,width:549,height:318});
