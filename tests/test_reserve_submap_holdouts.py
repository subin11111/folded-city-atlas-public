import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from reserve_submap_holdouts import reserve, canonical_sha256


def test_transitive_cross_connection_sensor_reuse_is_reserved():
    regions=[]
    for name,connection,groups,locked in [("a","x",["g1"],True),
                                          ("b","y",["g1","g2"],False),
                                          ("c","z",["g2"],False),
                                          ("d","z",["g3"],False)]:
        regions.append(dict(region_id=name,connection_id=connection,spatial_group=name,
                            correlation_group_ids=groups,locked_holdout=locked))
    edges=[dict(id=name,type="loop",role="heldout" if name=="a" else "train",region_group=name)
           for name in "abcd"]
    edges.append(dict(id="odom",type="odometry",role="train",region_group="a"))
    graph=dict(edges=edges,metadata={})
    view=dict(edges=edges,metadata={"region_validation_view":{
        "source_canonical_manifest_sha256":canonical_sha256(graph)}})
    result=reserve(graph,{"regions":regions},view)
    assert {e['id'] for e in result['edges'] if e['role']=='heldout'}=={'a','b','c'}
    assert graph['edges'][1]['role']=='train'
