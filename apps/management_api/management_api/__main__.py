import argparse
from pathlib import Path
import uvicorn
from management_api.app import create_app

parser = argparse.ArgumentParser(description='Start management; navigation requires explicit enable')
parser.add_argument('--root', type=Path, default=Path.cwd())
parser.add_argument('--host', default='127.0.0.1')
parser.add_argument('--port', type=int, default=8088)
parser.add_argument('--enable-navigation', action='store_true', help='Allow explicit A/B and stop requests after field preparation')
args = parser.parse_args()
uvicorn.run(create_app(args.root, navigation_enabled=args.enable_navigation), host=args.host, port=args.port)
