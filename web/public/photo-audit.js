const $=id=>document.getElementById(id);
const variantButton=document.createElement('button');variantButton.id='detailVariant';variantButton.textContent='기존 v3 보정본 보기';variantButton.hidden=true;$('detail').insertBefore(variantButton,$('original'));
const names={outside:'실외',lobby:'1층 로비',bridge_out1f:'입구 연결',floor2:'계단·2층',bridge_12_sub:'보조 계단'};
let library=[],filtered=[],page=0,inspecting=null,corrected=false;
function render(){
  filtered=library.filter(p=>($('session').value==='all'||p.session===$('session').value)&&
    ($('quality').value==='all'||$('quality').value==='eligible'&&p.navigation_eligible||$('quality').value==='unplaced'&&!p.navigation_eligible||$('quality').value==='flagged'&&p.quality_flags.length));
  const pages=Math.max(1,Math.ceil(filtered.length/100));page=Math.min(page,pages-1);$('grid').replaceChildren();
  for(const p of filtered.slice(page*100,page*100+100)){
    const button=document.createElement('button');button.className='card';
    const img=document.createElement('img');img.src='/'+p.thumb_url;img.loading='lazy';img.alt=`${names[p.session]} 원본 프레임 ${p.node}`;
    const body=document.createElement('div');const title=document.createElement('strong');title.textContent=`${names[p.session]} #${p.node}`;
    const info=document.createElement('small');info.textContent=`${p.width}×${p.height} · 지도 ${p.navigation_eligible?'후보':'보류'}`;
    const flag=document.createElement('small');flag.textContent=p.quality_flags.map(f=>({dark:'어두움',low_detail_or_blur:'낮은 세부/흐림',overexposed:'밝은 영역 많음'}[f]||f)).join(' · ')||'노출·선명도 주의 표시 없음';
    body.append(title,info,flag);button.append(img,body);button.addEventListener('click',()=>{
      inspecting=p;corrected=false;
      $('detailVariant').hidden=!p.display_url;$('detailVariant').textContent='기존 v3 보정본 보기';
      $('original').src='/'+p.url;$('original').alt='원본 DB에서 추출한 실제 RGB 사진';$('metadata').textContent=`${p.id}\n원본 크기: ${p.width} × ${p.height}\n선명도 지표: ${p.sharpness_laplacian_384}\n어두운 픽셀: ${(p.dark_fraction*100).toFixed(1)}%\n지도 대응 후보: ${p.navigation_eligible?'예':'아니오'}\n위치: ${p.position?.map(v=>v.toFixed(3)).join(', ')||'정합된 자세 없음'}\n방향: ${p.heading===undefined?'없음':(p.heading*180/Math.PI).toFixed(1)+'°'}\n원본 SHA-256: ${p.sha256}\n원근 사진 / 생성형 이미지 아님 / 360° 아님`;
      $('detail').showModal();
    });$('grid').appendChild(button);
  }
  $('resultCount').textContent=`${filtered.length.toLocaleString()}개`;$('page').textContent=`${page+1} / ${pages}`;
  $('previous').disabled=page===0;$('next').disabled=page>=pages-1;
}
try{
  const responses=await Promise.all(['photo_library','photo_audit','photo_coverage'].map(name=>fetch(`/data/${name}.json`).then(r=>{if(!r.ok)throw new Error(name+' 자료를 읽지 못했습니다.');return r.json();})));
  library=responses[0];const report=responses[1],coverage=responses[2];
  for(const [value,label]of[[report.total_image_records,'전체 RGB 레코드'],[report.decoded,'정상 해독'],[report.navigation_eligible,'지도 대응 후보'],[report.failures.length,'해독·처리 실패']]){
    const box=document.createElement('div');const number=document.createElement('strong');number.textContent=value.toLocaleString();box.append(number,document.createTextNode(label));$('stats').appendChild(box);
  }
  for(const [id,s] of Object.entries(report.sessions)){
    const option=document.createElement('option');option.value=id;option.textContent=`${names[id]} (${s.decoded})`;$('session').appendChild(option);
    const item=document.createElement('p');item.textContent=`${s.database} · ${s.decoded}개 해독 · 원본 DB 해시 ${s.database_sha256_before===s.database_sha256_after?'전후 동일':'불일치'}`;$('sources').appendChild(item);
    for(const path of s.contact_pages){const link=document.createElement('a');link.href='/'+path;link.target='_blank';link.rel='noreferrer';link.textContent=path.split('/').at(-1);$('sheets').appendChild(link);}
  }
  $('coverageSummary').textContent=`기본 최단 경로에서 약 0.75m 간격으로 뽑은 ${coverage.default_route.total_samples}개 표본 중, 요청 방향 사진 ${coverage.default_route.direction_matched}개, 방향을 바꾸면 볼 수 있는 사진까지 포함 ${coverage.default_route.any_direction}개입니다. 겹치는 표본과 같은 사진의 재사용이 있으며 전체 공간·모든 방향을 촬영했다는 뜻은 아닙니다.`;
  render();
}catch(error){$('error').hidden=false;$('error').textContent=error.message;}
for(const id of ['session','quality'])$(id).addEventListener('change',()=>{page=0;render();});
$('previous').addEventListener('click',()=>{page--;render();});$('next').addEventListener('click',()=>{page++;render();});$('close').addEventListener('click',()=>$('detail').close());
$('detailVariant').addEventListener('click',()=>{if(!inspecting?.display_url)return;corrected=!corrected;$('original').src='/'+(corrected?inspecting.display_url:inspecting.url);$('original').alt=corrected?'기존 v3 암부 보정 방식의 별도 파생 사진':'원본 DB에서 추출한 실제 RGB 사진';$('detailVariant').textContent=corrected?'원본 보기':'기존 v3 보정본 보기';});
