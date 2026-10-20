export const MEASURED_SESSIONS=Object.freeze({
  outside:{label:'실외 · 학생회관 입구 주변',source:'261016-234854-outside.db'},
  lobby:{label:'1층',source:'261017-000622-1f.db'},
  bridge_out1f:{label:'실외와 1층 연결',source:'261017-011331-bridgeout1f.db'},
  floor2:{label:'2층과 메인 계단',source:'261017-013501-2f.db'},
  bridge_12_sub:{label:'서브 계단',source:'261017-013904-bridg12_2.db'}
});

export async function loadMeasuredRecording(id='outside',fetcher=globalThis.fetch){
  const definition=Object.hasOwn(MEASURED_SESSIONS,id)?MEASURED_SESSIONS[id]:null;
  if(!definition)throw Error('등록된 학생회관 실측 구간을 선택해 주세요.');
  const response=await fetcher(`/data/gnss_${id}_recorded.json`);
  if(!response.ok)throw Error('학생회관 실측 GNSS 로그를 읽지 못했습니다.');
  const value=await response.json();
  if(value.schema!=='folded-city-gnss-v1'||value.simulated!==false||value.session!==id||value.source!==definition.source||!Array.isArray(value.records)||!value.records.length||!/^[a-f0-9]{64}$/.test(value.sha256||''))throw Error('원본 DB와 대응되는 실측 로그인지 확인할 수 없습니다.');
  return {...value,label:definition.label};
}
