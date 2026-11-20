import json
from pathlib import Path
import shutil
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1] / 'web'
AXES = ['X','Y','Z','ROLL','PITCH','YAW']
messages = []

def run(page, size):
    messages.clear()
    sockets = []
    def http(route):
        path = route.request.url.rsplit('/', 1)[-1] or 'index.html'
        content = (ROOT / path).read_text()
        mime = {'html':'text/html','css':'text/css','js':'application/javascript'}[path.rsplit('.',1)[-1]]
        route.fulfill(status=200, content_type=mime, body=content)
    page.route('https://example.test/**', http)
    def ws_handler(ws):
        sockets.append(ws)
        def send(data): ws.send(json.dumps(data))
        def incoming(raw):
            msg = json.loads(raw); messages.append(msg)
            if msg['type']=='hello':
                send({'type':'welcome','session_id':'test','role':msg['role'],'state':'READY'})
                send({'type':'room_state','revision':1,'state':'READY','modes':{'operator':True,'observer':True,'admin':True},'axes':{a:{'assigned':a == 'YAW','owner':'Otro equipo' if a == 'YAW' else None} for a in AXES}})
            elif msg['type']=='claim':
                leases=[{'axis':a,'lease_id':'test-'+a,'generation':1} for a in msg['axes']]
                send({'type':'claim_granted','revision':3,'leases':leases})
                # An older snapshot arriving after the grant must not revoke it.
                send({'type':'room_state','revision':2,'state':'READY','modes':{'operator':True,'observer':True,'admin':True},'axes':{a:{'assigned':False} for a in AXES}})
                send({'type':'room_state','revision':3,'state':'ACTIVE','modes':{'operator':True,'observer':True,'admin':True},'axes':{a:{'assigned':a in msg['axes'],'generation':1 if a in msg['axes'] else None} for a in AXES}})
        ws.on_message(incoming)
    page.route_web_socket('wss://example.test/ws', ws_handler)
    page.goto('https://example.test/')
    page.get_by_text('● Disponible').first.wait_for()
    def fits(label):
        dimensions=page.evaluate('''() => ({w:document.documentElement.scrollWidth, h:document.documentElement.scrollHeight, vw:innerWidth, vh:innerHeight, clipped:[...document.querySelectorAll('button:not([hidden])')].filter(e=>e.offsetParent && (e.getBoundingClientRect().right>innerWidth+1 || e.getBoundingClientRect().bottom>innerHeight+1)).map(e=>e.textContent)})''')
        assert dimensions['w']<=dimensions['vw'] and dimensions['h']<=dimensions['vh'] and not dimensions['clipped'], (size,label,dimensions)
        print(size,label,'sin desbordamiento')
    fits('modos')
    if size == (1280,720):
        page.screenshot(path='/tmp/teleop-modos-check.png')
    assert 'Disponible' in page.locator('#available-admin').inner_text()
    page.locator('[data-role="operator"]').click()
    assert page.locator('#modes').is_visible()
    assert page.locator('[data-role="operator"]').get_attribute('aria-pressed') == 'true'
    assert page.locator('#confirm-mode').is_enabled()
    assert [m['role'] for m in messages if m.get('type') == 'hello'] == ['observer']
    fits('rol seleccionado')
    page.locator('[data-role="operator"]').click()
    assert page.locator('.axis-option').nth(5).is_disabled()
    page.locator('.axis-option').nth(0).focus()
    page.keyboard.press('Space')
    assert page.locator('.axis-option').nth(0).evaluate('(e) => document.activeElement === e')
    sockets[-1].send(json.dumps({'type':'room_state','revision':1,'state':'READY','modes':{'operator':True,'observer':True,'admin':True},'axes':{a:{'assigned':a == 'YAW','owner':'Otro equipo' if a == 'YAW' else None} for a in AXES}}))
    page.wait_for_timeout(30)
    assert page.locator('.axis-option').nth(0).evaluate('(e) => document.activeElement === e')
    page.locator('.axis-option').nth(1).click()
    page.locator('.axis-option').nth(2).click()
    fits('selección')
    page.locator('#enter').click()
    page.locator('#operator-panel').wait_for(state='visible')
    assert page.locator('.control-row').count()==3
    fits('control')
    if size == (1280,720):
        page.screenshot(path='/tmp/teleop-control-check.png')
    move=page.locator('.control-row').first.locator('.move').last
    move.dispatch_event('pointerdown', {'pointerId':1,'button':0})
    page.wait_for_timeout(80)
    move.dispatch_event('pointerup', {'pointerId':1,'button':0})
    page.wait_for_timeout(80)
    inputs=[m for m in messages if m.get('type')=='input']
    assert any(m['values'].get('X')==1 for m in inputs)
    assert inputs[-1]['values'].get('X')==0
    print(size,'movimiento momentáneo vuelve a cero')
    page.locator('#home').click()
    page.get_by_text('● Disponible').first.wait_for()
    page.locator('[data-role="observer"]').click()
    assert page.locator('#modes').is_visible()
    page.locator('[data-role="observer"]').click()
    page.locator('#telemetry').wait_for(state='visible')
    fits('observador')
    page.locator('#home').click()
    page.get_by_text('● Disponible').first.wait_for()
    page.locator('[data-role="admin"]').click()
    assert page.locator('#modes').is_visible()
    # If another participant takes the mode before confirmation, the choice
    # remains visible and confirmation is disabled until it is available.
    sockets[-1].send(json.dumps({'type':'room_state','revision':2,'state':'READY','modes':{'operator':True,'observer':True,'admin':False},'axes':{a:{'assigned':False} for a in AXES}}))
    page.get_by_text('● Ocupado').wait_for()
    assert page.locator('#confirm-mode').is_disabled()
    sockets[-1].send(json.dumps({'type':'room_state','revision':3,'state':'READY','modes':{'operator':True,'observer':True,'admin':True},'axes':{a:{'assigned':False} for a in AXES}}))
    page.locator('#confirm-mode').wait_for(state='visible')
    page.get_by_text('● Disponible').last.wait_for()
    page.locator('#confirm-mode').click()
    page.locator('#admin-panel').wait_for(state='visible')
    fits('administrador')

with sync_playwright() as p:
    browser=p.chromium.launch(executable_path=shutil.which('google-chrome') or shutil.which('chromium'),headless=True,args=['--no-sandbox'])
    for size in [(1920,1080),(1280,720),(1024,600),(800,600)]:
        page=browser.new_page(viewport={'width':size[0],'height':size[1]})
        run(page,size)
        page.close()
    browser.close()
