import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('public_snapshot',Path(__file__).resolve().parents[1]/'scripts/public_snapshot.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
ROOT=Path(__file__).resolve().parents[1]

class PublicationTests(unittest.TestCase):
    def test_checkout_remote_formats_match_only_the_explicit_public_repository(self):
        name='subin11111/folded-city-atlas-public'
        for url in [f'https://github.com/{name}',f'https://github.com/{name}.git',f'git@github.com:{name}',f'git@github.com:{name}.git']:
            self.assertTrue(module.target_remote(url,name))
        for url in ['https://github.com/subin11111/folded-city-atlas.git',f'https://github.com/{name}-other.git']:
            self.assertFalse(module.target_remote(url,name))

    def test_only_explicit_source_roots_and_current_files_are_published(self):
        policy=json.loads((ROOT/'.public-export.json').read_text('utf-8'))
        for name in ['README.md','web/public/app.js','docs/experiments/result.md','src/optimise.py']:
            self.assertTrue(module.allowed(name,policy),name)
        for name in ['.git/config','.github/workflows/publish-public.yml','.env','web/.env.local',
                     'Folded_City_Codex_인계.md','docs/Folded_City_Codex_인계.md','docs/report.docx',
                     'web/public/data/raw.db','new_private_folder/notes.md','docs/key.pem']:
            self.assertFalse(module.allowed(name,policy),name)

    def test_secret_like_content_fails_closed_without_echoing_the_value(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'README.md';value='010'+'-1234'+'-5678';p.write_text(value)
            with self.assertRaises(ValueError) as caught:module.validate_content(p,'README.md')
            self.assertNotIn(value,str(caught.exception))

    def test_export_never_copies_deleted_history_or_untracked_files(self):
        policy=json.loads((ROOT/'.public-export.json').read_text('utf-8'));policy['required_files']=['README.md']
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'source';root.mkdir()
            def git(*args):return subprocess.run(['git','-C',str(root),*args],check=True,capture_output=True)
            git('init','-b','main');git('config','user.name','Test');git('config','user.email','test@example.invalid')
            private=root/'Folded_City_Codex_인계.md';private.write_text('old private content',encoding='utf-8')
            (root/'README.md').write_text('research',encoding='utf-8');git('add','.');git('commit','-m','old')
            git('rm','--','Folded_City_Codex_인계.md');git('commit','-m','remove')
            (root/'untracked.txt').write_text('not part of snapshot')
            out=Path(folder)/'export';result=module.export_snapshot(root,out,policy)
            self.assertEqual(result['files'],1);self.assertFalse((out/'.git').exists())
            self.assertEqual(sorted(p.name for p in out.iterdir()),['README.md'])

    def test_sync_refuses_any_checkout_other_than_the_public_target(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'repo';root.mkdir();snapshot=Path(folder)/'snapshot';snapshot.mkdir()
            subprocess.run(['git','-C',str(root),'init'],check=True,capture_output=True)
            subprocess.run(['git','-C',str(root),'remote','add','origin','https://github.com/subin11111/folded-city-atlas.git'],check=True)
            with self.assertRaises(ValueError):module.synchronise(snapshot,root,'subin11111/folded-city-atlas-public')

if __name__=='__main__':unittest.main()
