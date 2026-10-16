import fs from 'node:fs/promises';
import path from 'node:path';
import { Presentation, PresentationFile } from '@oai/artifact-tool';
import { finalizePresentation, applyPresentationChartFont } from '/home/just_grassy/.codex/plugins/cache/openai-primary-runtime/presentations/26.921.10847/skills/presentations/container_tools/artifact_tool_utils.mjs';
const root=path.resolve('.');
const tmp=path.join(root,'.presentation-build');
const out=path.join(root,'output/presentation');
const skill='/home/just_grassy/.codex/plugins/cache/openai-primary-runtime/presentations/26.921.10847/skills/presentations';
const family='Noto Sans CJK KR';
const p=Presentation.create({slideSize:{width:1280,height:720}});
const notes=[];
function text(s,t,x,y,w,h,size=26,bold=false,color='#171717'){
 const q=s.shapes.add({geometry:'textbox',position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});
 q.text=t;q.text.style={typeface:family,fontSize:size,bold,color,autoFit:'none'};return q;
}
async function img(s,n,x,y,w,h){s.images.add({blob:new Uint8Array(await fs.readFile(path.join(tmp,'assets',n+'.png'))),contentType:'image/png',alt:n,fit:'contain',position:{left:x,top:y,width:w,height:h}});}
function slide(title,dark=false){const s=p.slides.add();s.background.fill=dark?'#111111':'#FFFFFF';if(title)text(s,title,54,38,1172,65,44,true,dark?'#FFFFFF':'#111111');text(s,String(p.slides.items.length).padStart(2,'0'),1170,668,55,30,17,false,dark?'#AAAAAA':'#777777');return s;}
function note(s,title,sec,script,source){const t=`[발표 시간: ${sec}초]\n${script}\n\n[근거 자료]\n${source}`;s.speakerNotes.textFrame.setText(t);notes.push({title,sec,script});}
function cap(s,t,x,y,w,dark=false){text(s,t,x,y,w,40,17,false,dark?'#BBBBBB':'#666666');}

// 1
{
 const s=slide('',true);
 await img(s,'cover-x8',560,175,700,430);
 text(s,'FireDrone',56,133,650,100,76,true,'#FFFFFF');
 text(s,'로봇팔 소방 드론\n비행 제어 연구',60,250,655,155,48,true,'#FFFFFF');
 text(s,'R&E 2026  기술 컨설팅',60,475,550,44,27,false,'#D1D1D1');
 text(s,'HappyCoding & Airhood',60,533,550,37,22,false,'#AAAAAA');
 cap(s,'동축 X8 설계 시험 모델',790,614,425,true);
 note(s,'FireDrone',20,'안녕하세요. 저희는 로봇팔을 탑재한 소방 드론 FireDrone을 연구하고 있습니다. 건물과 주거 화재에 대응하는 것이 최종 목표입니다. 오늘은 현재 만든 하드웨어 설계와 비행 제어, 시뮬레이션 결과를 소개하고, 실제 기체로 옮기기 전에 필요한 검증 방향을 자문받고자 합니다.','README.md; simulation/simulator/README.md; 직접 실행한 X8 대시보드의 기체 부분. 시뮬레이션 모델이며 실물 사진이 아님.');
}
// 2
{
 const s=slide('연구 목표와 현재 개발 범위');
 text(s,'로봇팔이 움직이고 고장이 생겨도\n비행을 안정적으로 제어할 수 있을까?',56,127,1150,90,34,true);
 await img(s,'prop-scene',58,237,615,370);
 cap(s,'프로펠러 손상 시험의 X4 시각화',58,615,610);
 text(s,'현재 구현',736,257,470,46,28,true);
 text(s,'전원·FC 회로 설계\nSTM32 비행 제어 코드\nX4 검증 및 X8 시뮬레이터',736,311,490,138,27);
 text(s,'추가 통합',736,478,470,43,28,true);
 text(s,'LiDAR 인지, AI 판단, 소화 장치\n실제 기체 비행 및 소화 성능 검증',736,529,490,87,25);
 note(s,'연구 목표와 현재 개발 범위',25,'핵심 질문은 로봇팔의 움직임이나 고장이 비행 자세를 흔들 때, 제어기가 얼마나 안정적으로 대응할 수 있느냐입니다. 저장소에는 전원 회로와 비행 제어 보드 설계, STM32 제어 코드, 두 종류의 시뮬레이션 환경이 있습니다. 반면 AI 판단과 소화 장치를 포함한 전체 임무 통합은 앞으로 진행해야 합니다. 따라서 이번 컨설팅의 중심은 비행 기반 기술입니다.','README.md; simulation/simulator/out/viz_prop_damage.png; simulation/simulator/README.md. 그림은 X4 시뮬레이션이며 실물 기체가 아님.');
}
// 3
{
 const s=slide('하드웨어 설계: 전원 분배와 비행 제어');
 await img(s,'board',60,143,500,470);
 await img(s,'power',612,149,590,252);
 text(s,'BEC 전원 보드',62,606,490,45,28,true);
 text(s,'배터리 전원을 각 장치에 맞게 분배',616,419,596,42,27,true);
 text(s,'STM32 3.3 V\nRaspberry Pi·Arduino 5 V\n구동 장치용 12 V',616,482,565,116,26);
 cap(s,'검토 항목: 전류 용량, 발열, 전원 노이즈',616,618,590);
 note(s,'하드웨어 설계',30,'하드웨어는 장치별 전원 공급과 비행 제어 보드로 나누어 설계하고 있습니다. 왼쪽은 BEC 보드의 실제 PCB 설계 이미지이고, 오른쪽은 배터리 전원을 여러 전압으로 분배하는 회로도입니다. STM32에는 3.3볼트, Raspberry Pi와 Arduino에는 5볼트, 구동 장치에는 12볼트를 공급하도록 설계했습니다. 회로가 있다는 것과 전기적 성능이 검증됐다는 것은 다르므로, 부하 전류와 발열, 센서에 전달되는 전원 노이즈를 우선 확인하려 합니다.','PCB-Design/BEC/BEC-PCB.pdf; PCB-Design/BEC/BEC-Schematic.pdf 1쪽. 원본 색상을 유지하고 회로·보드 영역을 확대함.');
}
// 4
{
 const s=slide('비행 제어 구조');
 const rows=[['UKF','센서값으로 자세와 각속도 추정'],['ADRC · MRAC','외란 추정과 모델 변화 보정'],['INDI','필요한 모터 출력 변화량 계산'],['CAM','고장·출력 한계 안에서 추력 배분']];
 rows.forEach((r,i)=>{text(s,r[0],58,156+i*117,355,43,31,true);text(s,r[1],58,203+i*117,675,43,25);});
 await img(s,'ukf-chart',744,148,475,248);
 await img(s,'adrc-chart',744,409,475,248);
 cap(s,'X4 돌풍 시험: 자세 오차와 외란 추정',745,664,450);
 note(s,'비행 제어 구조',35,'비행 제어는 센서에서 시작해 모터 출력으로 끝납니다. 먼저 UKF가 센서의 잡음을 고려해 자세와 각속도를 추정합니다. ADRC는 예상하지 못한 외란을 추정하고, MRAC는 모델 변화에 대응하는 보정 역할을 맡습니다. INDI는 현재 움직임과 목표의 차이로 필요한 모터 출력 변화량을 계산합니다. 마지막 CAM은 고장 난 모터와 출력 한계를 고려해 명령을 배분합니다. 현재 제어 주기는 1밀리초입니다. 다만 알고리즘이 연결됐다는 사실과 실제 비행에서 충분한 성능을 보장한다는 것은 구분해 검증하고 있습니다.','simulation/simulator/README.md; flight_core/control.c; flight_core/allocator.c; drone-f405-foxeer-firmware/Control 및 Estimation; simulation/simulator/out/patched_ukf_gust.png의 UKF 자세 오차 및 ADRC 외란 추정 그래프 부분 확대. X4 저장 로그의 시각화. 축과 범례를 유지함.');
}
// 5
{
 const s=slide('로봇팔 움직임을 포함한 비행 시험',true);
 await img(s,'arm',52,139,900,505);
 text(s,'팔 반작용',987,170,240,43,28,true,'#FFFFFF');
 text(s,'기체를 흔드는\n토크를 주입',987,226,242,90,25,false,'#DDDDDD');
 text(s,'동시에 관찰',987,368,240,43,28,true,'#FFFFFF');
 text(s,'자세 변화\n비행 궤적\n모터 출력',987,428,239,135,26,false,'#DDDDDD');
 cap(s,'기존 X4/F405 시험 로그 재생 화면. 최신 X8 시험과 구분',54,648,1040,true);
 note(s,'로봇팔 외란 시험',30,'로봇팔을 움직이면 기체에도 반작용 토크가 생깁니다. 화면은 기존 X4 시험의 실제 로그를 재생한 것으로, 팔 외란이 있을 때 기체의 궤적과 자세, 모터 출력을 함께 볼 수 있습니다. 현재 X8 시뮬레이터에서는 단일 링크 팔의 각도와 가속도에 따른 반작용을 계산합니다. 하지만 실제 팔과 기체의 완전한 결합 운동까지 재현한 것은 아닙니다. 팔의 질량과 길이, 구동 속도를 실측해서 모델을 보정하는 것이 다음 과제입니다.','simulation/simulator/out/patched_arm_swing.csv; scripts/visualize.py --build patched --scenario arm_swing --frame 12.6 실행 화면; 최신 X8 팔 모델 한계는 README.md.');
}
// 6
{
 const s=slide('X4 재실행 결과: 수정 전 추락, 수정 후 착륙');
 async function csv(n){const rows=(await fs.readFile(path.join(tmp,'assets',n),'utf8')).trim().split('\n');const keys=rows.shift().split(',');return rows.map(r=>Object.fromEntries(r.split(',').map((v,i)=>[keys[i],Number(v)])));}
 const orig=await csv('nominal-original-now.csv'),cur=await csv('nominal-now.csv');
 const series=[['수정 전',orig,'#999999'],['수정 후',cur,'#111111']].map(([name,arr,color])=>{let a=arr.filter((_,i)=>i%80===0);if(a.at(-1)!==arr.at(-1))a.push(arr.at(-1));return{name,values:a.map(r=>r.pz),xValues:a.map(r=>r.t),line:{fill:color,width:3},marker:{symbol:'none'}};});
 const ch=s.charts.add('scatter',{position:{left:57,top:153,width:829,height:430},series,scatterOptions:{style:'line'},hasLegend:true,legend:{position:'bottom',textStyle:{typeface:family,fontSize:22,fill:'#222222'}},xAxis:{title:'시간 (s)',min:0,max:40,majorUnit:10,textStyle:{typeface:family,fontSize:20,fill:'#333333'},numberFormatCode:'0',majorGridlines:null},yAxis:{title:'고도 (m)',min:0,max:5,majorUnit:1,textStyle:{typeface:family,fontSize:20,fill:'#333333'},numberFormatCode:'0',majorGridlines:{fill:'#DDDDDD',width:1}},chartFill:'#FFFFFF',plotAreaFill:'#FFFFFF'});
 applyPresentationChartFont(ch,{fontFamily:family});
 text(s,'수정 전',950,170,260,42,28,true);text(s,'CRASHED',950,223,275,54,37,true);
 text(s,'수정 후',950,338,260,42,28,true);text(s,'LANDED',950,390,275,54,37,true);
 text(s,'정상 조건 / UKF 자세\n각 1회 재실행',950,501,275,82,23);
 text(s,'주요 수정: 외란 추정기 연결, UKF 수치 안정성, 모터 지연 반영',59,609,1160,46,26,true);
 cap(s,'정상 조건의 단일 시험 결과. 기존 복합 고장(gauntlet) 기록에는 실패가 남아 있음',59,659,1090);
 note(s,'X4 수정 전후 비교',40,'시뮬레이션은 제어 코드의 오류를 찾는 데 활용했습니다. 이번 자료를 위해 같은 정상 비행 조건에서 수정 전과 수정 후의 X4 프로그램을 각각 다시 실행했습니다. 회색 선은 수정 전으로, 이륙 후 고도를 잃고 추락했습니다. 검은 선은 수정 후로, 약 35초에 임무를 마치고 착륙했습니다. 수정 내용에는 외란 추정기의 잘못된 입력 연결, UKF의 수치 안정성, 실제 모터 지연을 무시하던 부분이 포함됩니다. 이 비교는 각 한 번의 시뮬레이션 결과입니다. 기존 로그에는 복합 고장 시험의 실패도 남아 있어, 모든 환경에서 안정적이라고 결론내리지는 않았습니다.','2026-10-07 직접 실행: firedrone_sim_orig --scenario nominal --out nominal-original-now.csv; firedrone_sim --scenario nominal --att ukf --out nominal-now.csv. 기존 빌드 실행. 각 1회, 기체는 X4 가정 모델. 곡선은 0.4초 간격 표본 및 마지막 표본을 사용. FINDINGS.md; out/summary.md의 gauntlet 실패 기록.');
}
// 7
{
 const s=slide('현재 X8 시뮬레이터 실행 화면',true);
 await img(s,'dashboard',55,126,896,560);
 text(s,'동축 X8',987,158,236,47,31,true,'#FFFFFF');
 text(s,'팔 4개\n상하 모터 8개',987,216,236,95,25,false,'#DDDDDD');
 text(s,'수동 앵글 모드',987,362,236,82,28,true,'#FFFFFF');
 text(s,'USB 조종 입력\n자세·고도·출력 표시',987,450,240,91,23,false,'#DDDDDD');
 text(s,'화면은 조종기 미연결\n시동 해제 상태',987,587,243,75,20,false,'#BBBBBB');
 note(s,'X8 시뮬레이터',35,'현재 개발의 중심은 네 개의 팔에 모터를 위아래로 배치한 동축 X8입니다. 화면은 이번에 직접 실행한 최신 대시보드입니다. USB 조종기의 스틱으로 목표 기울기와 추력을 주고, 자세와 고도, 팔의 움직임, 여덟 모터 출력을 확인하도록 만들었습니다. 지금 캡처는 조종기를 연결하지 않은 시동 해제 상태입니다. 자동 경로 비행 화면은 아닙니다. 모터 할당과 물리 모델, 고장 주입 관련 코드 테스트는 통과했지만, 실제 조종기 연결과 실제 기체 비행은 별도 검증이 필요합니다.','scripts/fpv.py --frames 15 --screenshot 실행; make test-x8 test-live 통과; simulation/simulator/README.md. X8 질량·관성·추력·팔 사양은 설계 시험용 가정값.');
}
// 8
{
 const s=slide('주변 인지와 모델링의 통합 상태');
 await img(s,'simulink-bw',53,167,606,366);
 cap(s,'저장된 Simulink 모델 미리보기',57,556,605);
 text(s,'Simulink',738,157,479,44,30,true);
 text(s,'제어기·기체·센서 모델 구성\nC 시뮬레이터와 교차 검증 필요',738,211,490,90,25);
 text(s,'LiDAR / AirSim',738,337,480,47,30,true);
 text(s,'점군의 2차원 표시 코드\n자동 장애물 회피는 추가 구현',738,391,492,92,25);
 text(s,'AI 화재 판단·소화 동작',738,514,494,46,28,true);
 text(s,'비행 제어와의 전체 통합이 다음 단계',738,569,488,89,25);
 note(s,'인지와 모델링 통합',25,'프로젝트에는 비행 제어 외의 확장 요소도 있습니다. 왼쪽은 저장된 Simulink 모델의 미리보기로, 센서와 제어기, 기체 동역학을 연결한 구조입니다. AirSim 코드는 LiDAR 점군을 받아 2차원으로 표시하는 단계입니다. 폴더 이름에 장애물 회피가 들어 있지만 현재 코드를 자율 회피 구현으로 소개하지는 않습니다. 앞으로 인지 결과를 비행 명령에 연결하고, AI 화재 판단과 소화 장치까지 통합해야 합니다.','simulation/drone-simulation-simulink/mainModels/flightSimulation.slx metadata/thumbnail.png. 썸네일에 MathWorks 저작권 표시가 있는 모델 자료이며 이번 작업에서 MATLAB 실행은 하지 않음. source/AirSimObstacleAvoidence/src/main.cpp; source/Lidar/src/LidarVisualization.cpp.');
}
// 9
{
 const s=slide('실기 검증으로 넘어가기 위한 과제');
 await img(s,'rig',64,138,400,477);
 cap(s,'시험 지그 참고 이미지\nBinary Box, 3D Flight Lab v1.2',62,625,440);
 text(s,'01  기체 파라미터 실측',535,153,680,45,29,true);
 text(s,'질량·관성, 추력 곡선, 동축 간섭',535,205,680,43,25);
 text(s,'02  제한된 환경에서 제어 시험',535,295,680,45,29,true);
 text(s,'팔 반작용, 센서 단절, 모터 고장 대응',535,347,680,43,25);
 text(s,'03  H7 보드에서 실행 검증',535,436,680,45,29,true);
 text(s,'드라이버 구현과 1 ms 주기 실행 시간 측정',535,488,680,47,25);
 text(s,'팔 선행 보상은 비활성, 단일 IMU 단절은 미해결\nH7 산출물은 제어 라이브러리 단계',535,563,680,84,23,true);
 note(s,'실기 검증 과제',30,'다음 단계는 가정값을 실측값으로 바꾸는 것입니다. 질량과 관성, 모터의 추력 곡선과 응답 지연, 동축 프로펠러의 간섭을 측정해야 합니다. 왼쪽은 저장소에 있는 외부 시험 지그 참고 자료이며 저희가 제작한 실물 사진은 아닙니다. 이런 제한된 시험 환경을 활용해 팔의 반작용과 고장 대응부터 확인하려 합니다. H7도 현재는 제어 라이브러리 단계여서, 센서와 ESC 드라이버를 연결한 후 실제 보드에서 제어 주기를 만족하는지 측정해야 합니다.','simulation/simulator/README.md; build/h7-core/libfiredrone.a의 역할 설명. 참고 이미지: Binary Box, 3D Flight Lab v1.2 Assembly and User Guide, PDF 4쪽. 저장소의 참고 설계이며 자체 제작 결과가 아님.');
}
// 10
{
 const s=slide('컨설팅에서 자문받고 싶은 내용',true);
 const questions=[['기체 설계','X8·10인치 프롭 계획에서 적정 중량과 추력 여유는?'],['제어 구조','현재 알고리즘 조합을 유지할지, 단순화할지?'],['검증 기준','어떤 고장 조건과 합격 기준부터 실험해야 할지?'],['개발 순서','H7 이식·팔 보상·인지 통합 중 무엇을 우선할지?']];
 questions.forEach((q,i)=>{text(s,q[0],61,155+i*116,280,42,28,true,'#FFFFFF');text(s,q[1],355,155+i*116,855,82,27,false,'#DDDDDD');});
 text(s,'목표: 실측에 근거한 설계와 단계별 비행 검증 계획 확정',61,642,1120,44,25,true,'#FFFFFF');
 note(s,'컨설팅 요청',30,'오늘 네 가지에 대해 자문을 부탁드리고 싶습니다. 첫째, 현재 X8과 10인치 프로펠러 계획에서 목표 중량과 추력 여유를 어떻게 정할지입니다. 둘째, 여러 제어 알고리즘을 함께 사용하는 현재 구조를 유지할지, 검증하기 쉬운 구조로 단순화할지입니다. 셋째, 고장 시험의 우선순위와 합격 기준을 어떻게 설정할지입니다. 마지막으로 H7 이식, 팔 보상, 인지 통합 중 어떤 순서가 적절할지 조언을 듣고 싶습니다. 컨설팅을 통해 실측과 비행 검증 계획을 구체화하겠습니다. 감사합니다.','질문은 simulation/simulator/README.md와 FINDINGS.md에 기록된 구현 범위·한계에서 도출. 10인치 프롭은 확정 성능이 아닌 계획.');
}
await fs.writeFile(path.join(out,'FireDrone_5min_발표대본.txt'),notes.map((n,i)=>`${i+1}. ${n.title} (${n.sec}초)\n${n.script}\n`).join('\n')+'\n총 계획 시간: 300초 (5분)\n');
const candidate=path.join(tmp,'candidate-color.pptx');
await (await PresentationFile.exportPptx(p)).save(candidate);
console.log('Draft exported');
await finalizePresentation({workspaceDir:root,candidatePath:candidate,finalPath:path.join(out,'FireDrone_컨설팅_5분_최종.pptx'),pythonExecutable:process.env.RUNTIME_PYTHON,integrityValidatorPath:path.join(skill,'container_tools/inspect_presentation_package_integrity.py'),layoutValidatorPath:path.join(skill,'container_tools/inspect_presentation_layout_geometry.py'),layoutArgs:['--expected-slide-size-emu','12192000,6858000','--validate-bullet-geometry','--validate-heading-fit'],requiredNativeChartOwnerSlides:[6],fontPolicy:{basis:'design',families:[family]},materializeLiteralChartWorkbooks:true,verifyArtifactToolImport:true,receiptPath:path.join(tmp,'validation-color.json')});
console.log('Finalized');
