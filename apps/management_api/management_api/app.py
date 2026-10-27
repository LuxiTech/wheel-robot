"""Management queries and explicitly enabled A/B navigation through SDK APIs."""
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal
from urllib.parse import urlsplit
from uuid import UUID
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict
from wheel_gateway.service import RuntimeGateway
from wheel_navigation.factory import create_navigation_service
from wheel_navigation.service import NavigationError


class GoCommand(BaseModel):
    model_config = ConfigDict(extra='forbid')
    point: Literal['A','B']
    command_id: UUID


class StopCommand(BaseModel):
    model_config = ConfigDict(extra='forbid')
    command_id: UUID


def create_app(root, *, navigation_enabled=False, navigation_service=None):
    root = Path(root).resolve()
    gateway = RuntimeGateway(root)
    navigation = navigation_service or create_navigation_service(root, enabled=navigation_enabled)

    @asynccontextmanager
    async def lifespan(app):
        navigation.start_worker()
        try:
            yield
        finally:
            navigation.close()

    app = FastAPI(title='Wheel Robot Management', version='0.3.0', lifespan=lifespan)
    app.state.navigation = navigation

    @app.exception_handler(NavigationError)
    async def navigation_error(request, exc):
        return JSONResponse({'detail':str(exc)}, status_code=exc.status)

    def control_origin(request):
        origin = request.headers.get('origin')
        if origin and urlsplit(origin).netloc != request.url.netloc:
            raise HTTPException(403, 'Control requests must use the same origin')

    @app.get('/api/v1/health')
    def health():
        return dict(service='management_api', status='ok', read_only=not navigation.enabled)

    app.get('/api/v1/status')(gateway.status)
    app.get('/api/v1/workflow')(gateway.workflow)
    app.get('/api/v1/configurations')(gateway.configurations)

    @app.get('/api/v1/runs')
    def runs():
        return gateway.runs()

    @app.get('/api/v1/navigation')
    def navigation_status():
        return navigation.snapshot()

    @app.get('/api/v1/devices')
    def devices():
        return navigation.snapshot()['devices']

    @app.get('/api/v1/navigation/commands/{command_id}')
    def navigation_command(command_id: UUID):
        with navigation.journal.transaction() as data:
            command = data['commands'].get(str(command_id))
            if command is None: raise HTTPException(404, 'Command was not recorded')
            return command

    @app.post('/api/v1/navigation/go')
    def navigation_go(command: GoCommand, request: Request):
        control_origin(request)
        result = navigation.navigate(command.point, str(command.command_id))
        code = 502 if result['state'] == 'UNKNOWN' else 409 if result['state'] == 'REJECTED' else 202
        return JSONResponse(result, status_code=code)

    @app.post('/api/v1/navigation/stop')
    def navigation_stop(command: StopCommand, request: Request):
        control_origin(request)
        result = navigation.stop(str(command.command_id))
        return JSONResponse(result, status_code=502 if result['state'] == 'UNKNOWN' else 202)

    app.mount('/', StaticFiles(directory=root/'apps/management_web', html=True), name='console')
    return app
