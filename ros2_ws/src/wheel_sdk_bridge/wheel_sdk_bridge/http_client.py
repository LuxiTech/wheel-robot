"""Bounded JSON HTTP transport for external SDK APIs; never retries writes."""
import json
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler
from urllib.error import HTTPError, URLError


class SDKError(RuntimeError):
    def __init__(self, message, status=502, uncertain=False):
        super().__init__(message)
        self.status = status
        self.uncertain = uncertain


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class JSONClient:
    def __init__(self, base_url, timeout=3.0):
        parsed = urlsplit(base_url)
        if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username or parsed.password:
            raise ValueError('SDK URL must be HTTP(S), without embedded credentials')
        if parsed.path not in ('', '/') or parsed.query or parsed.fragment:
            raise ValueError('SDK base URL must contain only scheme, host and port')
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def request(self, method, path, payload=None):
        if not path.startswith('/api/') or '://' in path:
            raise ValueError('Only SDK API paths are supported')
        body = None if payload is None else json.dumps(payload, allow_nan=False).encode('utf-8')
        request = Request(self.base_url + path, data=body, method=method,
                          headers={'Accept':'application/json', 'Content-Type':'application/json'})
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                data = response.read(2 * 1024 * 1024 + 1)
                if len(data) > 2 * 1024 * 1024:
                    raise SDKError('SDK response exceeds size limit', uncertain=method != 'GET')
                try:
                    result = json.loads(data)
                except (UnicodeError, ValueError) as exc:
                    raise SDKError('SDK returned invalid JSON', uncertain=method != 'GET') from exc
                if not isinstance(result, dict):
                    raise SDKError('SDK JSON must be an object', uncertain=method != 'GET')
                if result.get('ok') is False:
                    raise SDKError(str(result.get('error', 'SDK rejected request')), 409)
                return result
        except HTTPError as exc:
            message = 'SDK HTTP ' + str(exc.code)
            try:
                error = json.loads(exc.read(16384)).get('error', message)
                message = str(error.get('message', error)) if isinstance(error, dict) else str(error)
            except (ValueError, AttributeError):
                pass
            raise SDKError(message, 409 if exc.code in (400, 404, 409, 423) else 502,
                           uncertain=method != 'GET' and exc.code >= 500) from exc
        except (URLError, OSError, TimeoutError) as exc:
            raise SDKError('SDK connection failed: ' + str(exc), 503, uncertain=method != 'GET') from exc
