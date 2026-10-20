// Reference-informed palettes, not photographic facade reconstruction.
export const buildingAppearances={
  'way/367253036':{
    id:'student-brick-accent',wall:'#ddd3c3',accent:'#a36b50',joint:'#c4b4a0',glass:'#50676d',roof:'#737a78',pattern:'brick-accent',floorHeight:3.8,
    evidence:['photos/outside_19.jpg','photos/outside_263.jpg'],
    note:'직접 촬영한 입구의 적갈색 기둥, 밝은 벽체와 유리 색 계열을 단순화했다. 건물 전체 외벽의 실사 텍스처는 아니다.'
  },
  'way/367582526':{
    id:'library-granite',wall:'#d4d0c7',accent:'#bab6ae',joint:'#c0bcb3',glass:'#4d5e61',roof:'#858881',pattern:'stone-fins',floorHeight:4,
    evidence:['https://www.aurum.re.kr/Bits/BuildingDoc.aspx?num=178&page=1'],
    note:'건축 자료의 화강석과 복층유리 마감, 사진의 좁은 창과 수직 벽체를 참고한 표현이다.'
  },
  'way/471670359':{
    id:'new-engineering-glass',wall:'#c8cdd0',accent:'#b3bdc2',joint:'#aebbc2',glass:'#648c9e',roof:'#66777f',pattern:'curtain-wall',floorHeight:4,
    evidence:['https://popkon.konkuk.ac.kr/news/articleView.html?idxno=8654'],
    note:'건대신문 준공 사진의 회청색 유리와 밝은 수평 띠를 참고했다. 창 위치와 층수는 사진 측량값이 아니다.'
  }
};
const neutral={id:'unverified-neutral',wall:'#d8d6ce',accent:'#c6c4ba',joint:'#c5c3b9',glass:'#697c81',roof:'#838b89',pattern:'neutral',floorHeight:4,evidence:[],note:'외관 자료 미확인. 중립 재질로 표시한다.'};
export function appearanceFor(building){return buildingAppearances[building.id]||neutral;}
