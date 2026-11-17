"""Qt dashboard. AI/capture and UDP run outside the GUI event loop."""
from collections import deque
import json
import math
from pathlib import Path
import time
import signal

from PySide6.QtCore import Qt,QTimer,QRectF,QPointF,Signal,QEvent,QTranslator,QLibraryInfo
from PySide6.QtGui import QImage,QPainter,QColor,QPen,QFontDatabase
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QGridLayout,
    QFormLayout,QLabel,QPushButton,QGroupBox,QTabWidget,QScrollArea,QSplitter,QLineEdit,QCheckBox,
    QMessageBox,QTableWidget,QTableWidgetItem,QHeaderView,QComboBox,QPlainTextEdit,QProgressBar,QDoubleSpinBox)

from python_controller.dashboard_runtime import DashboardEngine,VisionStream,PreviewStream
from python_controller.visualization.korean import ko,add_choices,MEANINGS
from python_controller.diagnostics import NetworkProbe,diagnostic_report,save_report


def text(value,precision=3):
    if value is None: return '미수신·미확인'
    if isinstance(value,bool):return '예' if value else '아니오'
    if isinstance(value,float): return f'{value:.{precision}f}'
    if isinstance(value,(dict,list,tuple)): return json.dumps(value,ensure_ascii=False)
    return ko(str(value))


class Fields(QGroupBox):
    def __init__(self,title,keys):
        super().__init__(ko(title))
        layout=QFormLayout(self);layout.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        self.values={}
        for key in keys:
            name=QLabel(ko(key));name.setWordWrap(True)
            value=QLabel(ko('UNKNOWN'));value.setWordWrap(True);value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.values[key]=value;layout.addRow(name,value)

    def update_values(self,values):
        for name,value in values.items():
            if name in self.values:self.values[name].setText(ko(text(value)))


class CameraView(QWidget):
    selected=Signal(int)
    def __init__(self):
        super().__init__();self.setMinimumSize(320,240)
        self.image=None;self.people=();self.target=None;self.image_rect=QRectF()

    def set_frame(self,result,target):
        import cv2
        self.people=result['frame'].people;self.target=target
        image=result['image'].copy()
        h,w=image.shape[:2]
        cv2.line(image,(w//2,0),(w//2,h),(120,130,145),1)
        for person in self.people:
            box=tuple(map(int,person.bbox));color=(70,211,255) if person.track_id==target else (143,190,100)
            cv2.rectangle(image,box[:2],box[2:],color,2)
            cv2.putText(image,f'ID {person.track_id}  {person.confidence:.2f}',(box[0],max(20,box[1]-8)),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1,cv2.LINE_AA)
            if person.track_id==target:cv2.circle(image,((box[0]+box[2])//2,(box[1]+box[3])//2),5,color,-1)
        connections=((0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(5,9),(9,10),(10,11),(11,12),
                     (9,13),(13,14),(14,15),(15,16),(13,17),(0,17),(17,18),(18,19),(19,20))
        for points,owner in result['hands']:
            color=(70,211,255) if owner==target else (150,150,150)
            for a,b in connections:cv2.line(image,tuple(map(int,points[a])),tuple(map(int,points[b])),color,1)
            for x,y in points:cv2.circle(image,(int(x),int(y)),3,color,-1)
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.image=QImage(rgb.data,w,h,rgb.strides[0],QImage.Format_RGB888).copy()
        self.update()

    def paintEvent(self,event):
        painter=QPainter(self);painter.fillRect(self.rect(),QColor('#0b1119'))
        if self.image is None:
            painter.setPen(QColor('#9eafc4'));painter.drawText(self.rect(),Qt.AlignCenter,ko('Waiting for camera / model initialization'));return
        size=self.image.size();size.scale(self.size(),Qt.KeepAspectRatio)
        self.image_rect=QRectF((self.width()-size.width())/2,(self.height()-size.height())/2,size.width(),size.height())
        painter.drawImage(self.image_rect,self.image)

    def mousePressEvent(self,event):
        if self.image is None or not self.image_rect.contains(event.position()):return
        x=(event.position().x()-self.image_rect.x())*self.image.width()/self.image_rect.width()
        y=(event.position().y()-self.image_rect.y())*self.image.height()/self.image_rect.height()
        from python_controller.vision.person_tracker import TargetSelector
        chosen=TargetSelector.at_point(self.people,x,y)
        if chosen is not None:self.selected.emit(chosen)


class MotionArrow(QWidget):
    def __init__(self):
        super().__init__();self.setMinimumHeight(150);self.data=None
    def set_data(self,data):self.data=data;self.update()
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        center=QPointF(self.width()/2,self.height()/2+8)
        p.setPen(QPen(QColor('#314256'),1));p.drawLine(QPointF(center.x(),25),QPointF(center.x(),self.height()-10))
        p.drawLine(QPointF(25,center.y()),QPointF(self.width()-25,center.y()))
        p.setPen(QColor('#aab8c8'));p.drawText(12,18,ko('Command axes (proposed signs): +vx forward'))
        if not self.data:return
        for name,color in [('requested','#8195b4'),('safe','#4cdaa3')]:
            command=self.data.get(name)
            if command is None:continue
            dx=command['vy_cm_s']/5*55;dy=-command['vx_cm_s']/5*55
            length=math.hypot(dx,dy)
            if length>65:dx*=65/length;dy*=65/length
            end=QPointF(center.x()+dx,center.y()+dy)
            p.setPen(QPen(QColor(color),3));p.drawLine(center,end)
            if length>1:
                angle=math.atan2(dy,dx)
                for off in (-.6,.6):p.drawLine(end,QPointF(end.x()-10*math.cos(angle+off),end.y()-10*math.sin(angle+off)))
        p.setPen(QColor('#4cdaa3'));p.drawText(12,self.height()-8,ko(f'Safe rotation: {self.data["safe"]["w_rad_s"]:+.3f} rad/s'))


class MotorGraph(QWidget):
    def __init__(self):
        super().__init__();self.setMinimumHeight(270);self.history=deque(maxlen=250);self.last_rx=None
    def observe(self,data):
        tr,m=data['transport'],data['motors']
        if tr.get('telemetry_valid') and tr.get('last_rx_at')!=self.last_rx and m['target_rpm'] is not None and m['measured_rpm'] is not None:
            self.last_rx=tr['last_rx_at'];self.history.append((self.last_rx,m['target_rpm'],m['measured_rpm']))
        self.update()
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        if not self.history:
            p.setPen(QColor('#aab8c8'));p.drawText(self.rect(),Qt.AlignCenter,ko('NO MOTOR MEASUREMENTS RECEIVED\nTarget and measured RPM history appears after valid ESP32 telemetry.'));return
        end=self.history[-1][0]
        points=[r for r in self.history if r[0]>=end-30]
        row=self.height()/3
        for motor in range(3):
            y=motor*row
            limit=max(1,max(abs(v) for _,a,b in points for v in (a[motor],b[motor])))
            p.setPen(QColor('#aab8c8'));p.drawText(8,int(y+17),ko(f'M{motor+1} RPM  +/-{limit:.1f}   target blue / measured green'))
            for column,color in [(1,'#62a5ff'),(2,'#4cdaa3')]:
                p.setPen(QPen(QColor(color),1.7));previous=None
                for point in points:
                    current=QPointF(10+(point[0]-(end-30))/30*(self.width()-20),y+row*.60-point[column][motor]/limit*row*.28)
                    if previous is not None:p.drawLine(previous,current)
                    previous=current


def scroll(widget):
    area=QScrollArea();area.setWidgetResizable(True);area.setWidget(widget);area.setFrameShape(QScrollArea.NoFrame)
    return area


class Dashboard(QMainWindow):
    def __init__(self,config,hardware=False,preview=False,folder=None,stream=None,engine=None):
        super().__init__()
        # Restricted/offscreen Windows sessions can expose an empty font database.
        if not QFontDatabase.families():
            for name in ('segoeui.ttf','segoeuib.ttf','malgun.ttf','malgunbd.ttf'):
                font=Path('C:/Windows/Fonts')/name
                if font.exists():QFontDatabase.addApplicationFont(str(font))
        self.setWindowTitle(ko('Omni Robot | Validation Console'))
        app=QApplication.instance()
        if not hasattr(app,'_korean_translator'):
            app._korean_translator=QTranslator(app)
            app._korean_translator.load('qtbase_ko',QLibraryInfo.path(QLibraryInfo.TranslationsPath))
            app.installTranslator(app._korean_translator)
        self.resize(1320,920);self.setMinimumSize(940,680)
        root=Path(__file__).resolve().parents[2]
        self.connection_file=root/'settings'/'connection.json'
        source='SYNTHETIC_PREVIEW' if preview else 'LIVE_CAMERA'
        self.engine=engine or DashboardEngine(config,folder or root/'logs',source)
        self.stream=stream or (PreviewStream() if preview else VisionStream(config))
        self.last_image_at=None;self.last_data=None;self.closing=False
        self.network_probe=NetworkProbe();self.last_diagnostic_report=None
        self.build_ui()
        self.apply_style()
        if hardware:
            try:self.engine.connect_motor_test(config['communication']['host'],True)
            except Exception as exc:
                self.engine.notice=f'HARDWARE CONNECTION FAILED: {exc}'
                self.engine.c.halt('FAULT','HARDWARE_CONNECTION_FAILED',True)
            self.tabs.setCurrentIndex(self.motor_tab_index)
        self.stream.start()
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(50)

    def build_ui(self):
        central=QWidget();outer=QVBoxLayout(central);outer.setContentsMargins(16,12,16,12);outer.setSpacing(10)
        top=QHBoxLayout();title=QLabel(ko('OMNI / VALIDATION CONSOLE'));title.setObjectName('title')
        self.mode=QLabel(ko('SIMULATION'));self.mode.setObjectName('mode')
        self.state=QLabel(ko('STARTING'));self.arm_state=QLabel(ko('DISARMED'))
        self.estop=QPushButton(ko('EMERGENCY STOP   [SPACE]'));self.estop.setObjectName('estop');self.estop.setMinimumHeight(46)
        self.estop.clicked.connect(lambda:self.action('ESTOP'))
        top.addWidget(title);top.addStretch();top.addWidget(self.mode);top.addWidget(self.state);top.addWidget(self.arm_state);top.addWidget(self.estop)
        outer.addLayout(top)
        self.pipeline=QLabel(ko('Initializing camera and models in background...'));self.pipeline.setWordWrap(True)
        outer.addWidget(self.pipeline)
        actions=QHBoxLayout()
        for label,action in [('Confirm target [Enter]','CONFIRM'),('Arm [A]','ARM'),('Resume [R]','RESUME'),('Pause [P]','PAUSE'),('Clear target [C]','CLEAR')]:
            button=QPushButton(ko(label));button.setObjectName(action);button.clicked.connect(lambda checked=False,a=action:self.action(a));actions.addWidget(button)
        self.distance=QLabel(ko('Relative height target: --'));actions.addWidget(self.distance);actions.addStretch();outer.addLayout(actions)
        self.tabs=QTabWidget();outer.addWidget(self.tabs,1)
        # The primary camera view stays large; long details scroll independently.
        split=QSplitter(Qt.Horizontal)
        left=QWidget();ll=QVBoxLayout(left);ll.setContentsMargins(0,0,4,0)
        self.camera=CameraView();self.camera.selected.connect(lambda id:self.engine.request('SELECT',id));ll.addWidget(self.camera,1)
        self.camera_status=QLabel(ko('Waiting for frames'));self.camera_status.setWordWrap(True);ll.addWidget(self.camera_status)
        self.tracking=Fields('TARGET / TRACKING',['People','Target ID','Confirmed / Lost','Confidence','Center (px)','Last seen / age (s)','Horizontal error','Relative height'])
        ll.addWidget(self.tracking)
        right=QWidget();rl=QVBoxLayout(right);rl.setContentsMargins(4,0,0,0)
        self.raw=Fields('01 / RAW GESTURE',['Hands','Raw labels / confidence / owner','Sample age (s)'])
        self.confirmed=Fields('02 / CONFIRMED GESTURE',['Current confirmation','Time / required (s)','Frames / required','Confidence threshold'])
        self.progress=QProgressBar();self.progress.setRange(0,100);self.progress.setValue(0)
        self.confirmed.layout().addRow(self.progress)
        self.applied=Fields('03 / APPLIED ACTION',['Applied this update','Last gesture action / age (s)','Rejection / hold reason'])
        self.safe=Fields('CONTROL / SAFETY',['State','Requested vx / vy / w','Safe vx / vy / w','Limits or stop applied','Final stop / status','Radar'])
        for widget in (self.raw,self.confirmed,self.applied,self.safe):rl.addWidget(widget)
        self.arrow=MotionArrow();rl.addWidget(self.arrow);rl.addStretch()
        split.addWidget(left);split.addWidget(scroll(right));split.setSizes([780,430]);self.tabs.addTab(split,ko('DASHBOARD'))
        self.vision_text=QPlainTextEdit();self.vision_text.setReadOnly(True)
        self.tabs.addTab(self.vision_text,ko('VISION'))
        motion=QWidget();ml=QVBoxLayout(motion)
        self.telemetry=Fields('ESP32 / TRANSPORT',['Connection','Receive validity / reason','Last RX / age (s)','Session / telemetry last_seq','Last successful UDP send','Telemetry seq match (not ACK)','ESP32 state / fault','ESP32 reported desired cmd','Post-limit body command','Stop transmission / feedback'])
        ml.addWidget(self.telemetry)
        self.motor_table=QTableWidget(3,6);self.motor_table.setHorizontalHeaderLabels([ko(s) for s in ['Motor','Target cm/s','Measured cm/s','Target RPM*','Measured RPM*','PWM / encoder']])
        self.motor_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);self.motor_table.setMaximumHeight(155)
        self.motor_table.setEditTriggers(QTableWidget.NoEditTriggers);ml.addWidget(self.motor_table)
        ml.addWidget(QLabel(ko('* RPM converted using configured wheel radius. Encoders do not prove floor motion.')))
        self.graph=MotorGraph();ml.addWidget(self.graph,1);self.tabs.addTab(scroll(motion),ko('MOTION'))
        tests=QWidget();tl=QVBoxLayout(tests)
        connect=QGroupBox(ko('ESP32 CONNECTION / STOP proof then stationary heartbeat'));cl=QVBoxLayout(connect)
        row=QHBoxLayout();self.host=QLineEdit();self.host.setPlaceholderText(ko('ESP32 IPv4 address'))
        from python_controller.communication.motor_test import load_connection
        self.host.setText(ko(self.engine.cfg['communication']['host'] or load_connection(self.connection_file)))
        self.authorize=QCheckBox(ko('Authorize UDP connection only (does not authorize motor motion)'))
        connect_button=QPushButton(ko('Connect UDP'));connect_button.clicked.connect(self.connect_hardware)
        save_ip=QPushButton(ko('Save IP'));save_ip.clicked.connect(self.save_ip)
        disconnect_button=QPushButton(ko('Stop + return to SIMULATION'));disconnect_button.clicked.connect(self.disconnect)
        row.addWidget(self.host);row.addWidget(save_ip);row.addWidget(connect_button);row.addWidget(disconnect_button);cl.addLayout(row);cl.addWidget(self.authorize)
        cl.addWidget(QLabel(ko('TX 8888 / RX 8889. No separate ACK packet. Connection sends no cmd_vel. Automatic hardware following is disabled.')))
        self.log_path=QLabel(ko(str(self.engine.logger.directory) if self.engine.logger else 'Logging disabled (test)'));self.log_path.setWordWrap(True)
        tl.addWidget(QLabel(ko('JSONL session log (10 Hz + operator events; no camera images):')));tl.addWidget(self.log_path)
        self.test_table=QTableWidget(7,3);self.test_table.setHorizontalHeaderLabels([ko(s) for s in ['Validation step','Operator result','Evidence / notes']])
        self.test_table.horizontalHeader().setSectionResizeMode(0,QHeaderView.ResizeToContents)
        self.test_table.horizontalHeader().setSectionResizeMode(2,QHeaderView.Stretch)
        self.test_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for i,row in enumerate(self.engine.book.rows):
            for j,value in enumerate((row['test'],row['result'],row['notes'])):self.test_table.setItem(i,j,QTableWidgetItem(ko(value)))
        tl.addWidget(self.test_table)
        controls=QHBoxLayout();self.test_index=QComboBox();add_choices(self.test_index,self.engine.book.NAMES)
        self.test_result=QComboBox();add_choices(self.test_result,['NOT TESTED','PASS','FAIL'])
        self.test_notes=QLineEdit();self.test_notes.setPlaceholderText(ko('Observed result / evidence (required for PASS or FAIL)'))
        self.performed=QCheckBox(ko('I performed this step'))
        record=QPushButton(ko('Record result'));record.clicked.connect(self.record_result)
        controls.addWidget(self.test_index);controls.addWidget(self.test_result);controls.addWidget(self.test_notes,1)
        tl.addLayout(controls);tl.addWidget(self.performed);tl.addWidget(record)
        guide=QLabel(ko('1 Camera: image / latency.  2 Tracking: select, move, hide.  3 Gestures: palm, fist, up, down.\n'
                    '4 Follow: left/right/near/far/lost.  5 ESP32: TX, telemetry seq/echo, feedback, disconnect.\n'
                    '6 Lifted wheels: directions, encoders, RPM, STOP.  7 Floor drive: separate user approval and controlled space.\n'
                    'Recording a result does not execute a motor test. All tests start NOT TESTED. No rear radar is connected.'))
        guide.setWordWrap(True);tl.addWidget(guide);tl.addStretch();self.tabs.addTab(scroll(tests),ko('TEST & LOG'))
        bench=QWidget();bl=QVBoxLayout(bench);bl.addWidget(connect)
        diagnostic_box=QGroupBox('연결 및 통신 진단 · 모터를 움직이지 않습니다');dl=QVBoxLayout(diagnostic_box)
        diagnostic_buttons=QHBoxLayout()
        self.network_check=QPushButton('네트워크·수신 포트 점검');self.network_check.clicked.connect(self.run_network_check)
        self.save_diagnosis=QPushButton('진단 보고서 저장 (JSON·Markdown)');self.save_diagnosis.clicked.connect(self.export_diagnosis)
        diagnostic_buttons.addWidget(self.network_check);diagnostic_buttons.addWidget(self.save_diagnosis);dl.addLayout(diagnostic_buttons)
        self.network_status=QLabel('네트워크 점검은 아직 수행하지 않았습니다. IP 저장과 UDP 연결은 별도입니다.');self.network_status.setWordWrap(True);dl.addWidget(self.network_status)
        self.diagnostic_steps=QTableWidget(6,3);self.diagnostic_steps.setHorizontalHeaderLabels(['진단 단계','결과','관측 근거 및 다음 확인'])
        self.diagnostic_steps.horizontalHeader().setSectionResizeMode(0,QHeaderView.ResizeToContents)
        self.diagnostic_steps.horizontalHeader().setSectionResizeMode(1,QHeaderView.ResizeToContents)
        self.diagnostic_steps.horizontalHeader().setSectionResizeMode(2,QHeaderView.Stretch)
        self.diagnostic_steps.setEditTriggers(QTableWidget.NoEditTriggers);self.diagnostic_steps.setMinimumHeight(290);dl.addWidget(self.diagnostic_steps)
        dl.addWidget(QLabel('현재 IP는 시리얼 모니터 115200bps에서 STATUS로 확인하세요. 수동 모드라면 먼저 STOP → AUTO → STATUS를 확인합니다.'))
        dl.addWidget(QLabel('송수신 상세 원문: 프로토콜 식별자·JSON 필드명은 호환성을 위해 유지하며, 자격 증명 필드는 제외합니다.'))
        self.communication_text=QPlainTextEdit();self.communication_text.setReadOnly(True);self.communication_text.setMaximumHeight(160);dl.addWidget(self.communication_text)
        bl.addWidget(diagnostic_box)
        warning=QLabel(ko('LIFT ALL THREE WHEELS OFF THE FLOOR. Bench tests only. Camera/gesture motion output is disconnected.\n'
                       'Forward/left/rotation labels use proposed body axes; verify physical signs. One authorization allows one short burst.'))
        warning.setWordWrap(True);bl.addWidget(warning)
        self.bench_state=Fields('MOTOR TEST / transmission is not proof of rotation',
            ['Source','Status / reason','Remaining (s)','Manual command vx / vy / w','Command processing evidence','Stop feedback','Current / last trial'])
        bl.addWidget(self.bench_state)
        confirmations=QVBoxLayout();self.lifted=QCheckBox(ko('All wheels are lifted and the platform is secured'))
        self.burst_approval=QCheckBox(ko('Authorize ONE motor test burst'))
        confirmations.addWidget(self.lifted);confirmations.addWidget(self.burst_approval);bl.addLayout(confirmations)
        settings=QGridLayout();self.burst_duration=QDoubleSpinBox();self.burst_duration.setRange(.1,self.engine.cfg['motor_test']['max_duration_s'])
        self.burst_duration.setSingleStep(.1);self.burst_duration.setValue(self.engine.cfg['motor_test']['duration_s']);self.burst_duration.setSuffix(' 초')
        self.manual_arm=QPushButton(ko('Enable one MOTOR TEST'));self.manual_arm.setEnabled(False);self.manual_arm.clicked.connect(self.enable_motor_test)
        self.heartbeat_button=QPushButton(ko('Stationary heartbeat check'));self.heartbeat_button.setEnabled(False);self.heartbeat_button.clicked.connect(self.probe_heartbeat)
        settings.addWidget(QLabel(ko(f'SLOW: {self.engine.cfg["motor_test"]["linear_cm_s"]} cm/s / {self.engine.cfg["motor_test"]["angular_rad_s"]} rad/s | duration')),0,0)
        settings.addWidget(self.burst_duration,0,1);settings.addWidget(self.manual_arm,1,0);settings.addWidget(self.heartbeat_button,1,1);bl.addLayout(settings)
        buttons=QGridLayout();self.test_buttons={}
        for label,name,row,col in [('Forward (+vx)','FORWARD',0,1),('Backward (-vx)','BACKWARD',2,1),('Left (+vy)','LEFT',1,0),
                                   ('Right (-vy)','RIGHT',1,2),('Turn left (+w)','TURN_LEFT',0,0),('Turn right (-w)','TURN_RIGHT',0,2)]:
            button=QPushButton(ko(label));button.setEnabled(False);button.clicked.connect(lambda checked=False,n=name:self.start_motor_test(n))
            self.test_buttons[name]=button;buttons.addWidget(button,row,col)
        stop_button=QPushButton(ko('STOP'));stop_button.setObjectName('estop');stop_button.clicked.connect(lambda:self.action('PAUSE'));buttons.addWidget(stop_button,1,1)
        bl.addLayout(buttons)
        self.burst_progress=QProgressBar();self.burst_progress.setRange(0,100);bl.addWidget(self.burst_progress)
        self.bench_motors=QTableWidget(3,7);self.bench_motors.setHorizontalHeaderLabels([ko(s) for s in ['Motor','Target cm/s','Measured cm/s','Measured RPM*','PWM','Encoder count','Trial encoder evidence']])
        self.bench_motors.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);self.bench_motors.setMaximumHeight(150)
        self.bench_motors.setEditTriggers(QTableWidget.NoEditTriggers);bl.addWidget(self.bench_motors)
        self.trial_selection=QComboBox();self.trial_selection.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon);bl.addWidget(self.trial_selection)
        observations=QGridLayout();self.direction_observations=[]
        for motor in range(3):
            box=QComboBox();add_choices(box,['NOT TESTED','MATCHES EXPECTED','OPPOSITE','NO ROTATION'])
            observations.addWidget(QLabel(ko(f'M{motor+1} visual result')),motor,0);observations.addWidget(box,motor,1);self.direction_observations.append(box)
        bl.addLayout(observations)
        self.motor_notes=QLineEdit();self.motor_notes.setPlaceholderText(ko('Physical observations / wiring / encoder direction notes; do not infer body travel from encoder counts'))
        bl.addWidget(self.motor_notes)
        self.hardware_checks=[]
        for label in ['모터 전원 전압을 실제 측정함','드라이버 STBY와 공통 GND를 확인함','모터 핀·엔코더 배선·풀업을 확인함','물리 전원 차단 수단을 확인함']:
            check=QCheckBox(label);self.hardware_checks.append(check);bl.addWidget(check)
        bl.addWidget(QLabel('위 점검은 사용자 관찰 기록이며 프로그램이 전압·배선을 자동 검증한 결과가 아닙니다.'))
        record_motor=QPushButton(ko('Record visual observations for selected completed trial'));record_motor.clicked.connect(self.record_motor_observations);bl.addWidget(record_motor)
        self.trial_text=QPlainTextEdit();self.trial_text.setReadOnly(True);self.trial_text.setMaximumHeight(170);bl.addWidget(self.trial_text)
        bl.addStretch();self.motor_tab_index=self.tabs.addTab(scroll(bench),ko('MOTOR TEST'))
        self.setCentralWidget(central)
        QApplication.instance().installEventFilter(self)

    def eventFilter(self,watched,event):
        if event.type()==QEvent.KeyPress and isinstance(watched,QWidget) and (watched is self or self.isAncestorOf(watched)):
            editing=isinstance(QApplication.focusWidget(),(QLineEdit,QPlainTextEdit,QComboBox,QTableWidget))
            if event.key()==Qt.Key_Space:
                self.action('ESTOP')
                return not isinstance(QApplication.focusWidget(),(QLineEdit,QPlainTextEdit))
            if not editing and not event.modifiers() & (Qt.ControlModifier|Qt.AltModifier|Qt.MetaModifier):
                action={Qt.Key_E:'ESTOP',Qt.Key_A:'ARM',Qt.Key_R:'RESUME',Qt.Key_P:'PAUSE',Qt.Key_C:'CLEAR',Qt.Key_Return:'CONFIRM',Qt.Key_Enter:'CONFIRM'}.get(event.key())
                if action:
                    if not event.isAutoRepeat():self.action(action)
                    return True
                if event.key() in (Qt.Key_Escape,Qt.Key_Q):self.close();return True
        return super().eventFilter(watched,event)

    def action(self,action):
        self.engine.request(action)
        if action=='ESTOP':self.pipeline.setText(ko('EMERGENCY STOP LATCHED / explicit re-arm required'))

    def warning(self,title,message):
        original=str(message);translated=ko(original)
        if translated==original and not any('가'<=c<='힣' for c in translated):
            translated='작업을 완료하지 못했습니다. 입력·연결·승인 상태를 확인하세요.\n세부 오류 원문: '+original
        QMessageBox.warning(self,ko(title),translated)

    def run_network_check(self):
        try:
            host=self.host.text().strip()
            if self.engine.client and host!=self.engine.client.host:
                raise ValueError('현재 연결 주소와 입력 주소가 다릅니다. 먼저 연결을 해제하고 새 주소를 점검하세요.')
            self.network_probe.start(host,bool(self.engine.client and self.engine.client.snapshot().get('rx_bound')))
        except Exception as exc:self.warning('네트워크 점검 시작 실패',str(exc))

    def export_diagnosis(self):
        try:
            if not self.last_diagnostic_report:raise ValueError('먼저 진단 상태를 갱신하세요.')
            report=dict(self.last_diagnostic_report)
            report.update(program_mode=self.engine.mode,evidence_source=self.engine.source,
                          operator_hardware_checks={c.text():('OPERATOR_CHECKED' if c.isChecked() else 'NOT TESTED') for c in self.hardware_checks})
            folder=self.engine.logger.directory if self.engine.logger else Path(__file__).resolve().parents[2]/'logs'
            path=save_report(folder,report)
            self.engine.notice=f'진단 보고서를 저장했습니다: {path}'
        except Exception as exc:self.warning('진단 보고서 저장 실패',str(exc))

    def connect_hardware(self):
        try:
            self.engine.connect_motor_test(self.host.text().strip(),self.authorize.isChecked())
            self.authorize.setChecked(False)
        except Exception as exc:self.warning('Hardware connection refused',str(exc))

    def disconnect(self):
        self.engine.disconnect();self.authorize.setChecked(False)
        self.lifted.setChecked(False);self.burst_approval.setChecked(False)

    def save_ip(self):
        try:
            from python_controller.communication.motor_test import save_connection
            save_connection(self.connection_file,self.host.text().strip())
            self.engine.notice='ESP32 IP saved locally; no network connection started'
        except Exception as exc:self.warning('IP not saved',str(exc))

    def enable_motor_test(self):
        try:
            self.engine.authorize_motor_test(self.lifted.isChecked(),self.burst_approval.isChecked())
            self.burst_approval.setChecked(False)
        except Exception as exc:self.warning('Motor test not enabled',str(exc))

    def probe_heartbeat(self):
        try:self.engine._manual_client().probe_heartbeat()
        except Exception as exc:self.warning('Heartbeat not started',str(exc))

    def start_motor_test(self,direction):
        try:self.engine.begin_motor_test(direction,self.burst_duration.value())
        except Exception as exc:self.warning('Motor test not started',str(exc))

    def record_motor_observations(self):
        try:
            self.engine._manual_client().record_observations(self.trial_selection.currentData(),
                [box.currentData() for box in self.direction_observations],self.motor_notes.text())
        except Exception as exc:self.warning('Observations not recorded',str(exc))

    def record_result(self):
        index,result=self.test_index.currentIndex(),self.test_result.currentData()
        if result!='NOT TESTED' and not self.performed.isChecked():
            self.warning('Result not recorded','Confirm you actually performed this step.');return
        if result=='PASS' and index>=4 and (self.engine.mode!='HARDWARE' or self.engine.source!='LIVE_CAMERA'):
            self.warning('Result not recorded','A hardware PASS cannot be recorded from SIMULATION or synthetic preview.');return
        try:
            self.engine.book.record(index,result,self.test_notes.text(),self.engine.mode,
                                    'OPERATOR_RECORDED / '+self.engine.source)
            if self.engine.logger:
                self.engine.logger.save_tests(self.engine.book)
                self.engine.logger.write(dict(event='OPERATOR_TEST_RESULT',test_result=dict(self.engine.book.rows[index])),True)
            for j,value in enumerate((self.engine.book.rows[index]['test'],result,self.test_notes.text())):
                self.test_table.item(index,j).setText(ko(value))
            self.performed.setChecked(False)
        except Exception as exc:self.warning('Result not recorded',str(exc))

    def poll(self):
        try:
            result=self.stream.latest()
            health=dict(vision=self.stream.status,camera=getattr(self.stream,'camera_status',None))
            data=self.engine.tick(result,pipeline_error=self.stream.error or self.engine.error,pipeline_health=health)
            self.stream.target_id=self.engine.c.target_id
            self.update_display(data,result)
        except Exception as exc:
            self.engine.request('ESTOP');self.pipeline.setText(ko(f'UI ERROR / STOP LATCHED: {type(exc).__name__}: {exc}'))

    def update_display(self,d,result):
        self.last_data=d;now=time.monotonic();tr=d['transport'];g=d['gesture'];v=d['vision']
        manual=d.get('control_source')=='MANUAL_MOTOR_TEST'
        self.mode.setText(('실제 장치 · 수동 시험' if manual else '시뮬레이션')+(' · 합성 (SYNTHETIC)' if self.engine.source!='LIVE_CAMERA' else ''))
        self.mode.setProperty('hardware',d['program_mode']=='HARDWARE');self.mode.style().unpolish(self.mode);self.mode.style().polish(self.mode)
        self.state.setText(MEANINGS.get(d['motor_test']['status'] if manual else d['state'],d['state']))
        self.arm_state.setText(ko(('MANUAL ENABLED' if manual else 'ARMED') if d['armed'] else 'DISARMED'))
        for action in ('ARM','RESUME'):
            self.findChild(QPushButton,action).setEnabled(not manual)
        restriction='MULTIPLE PERSONS DETECTED - GESTURE CONTROL BLOCKED' if d['detected_people']>1 else ''
        perception='VISION DISCONNECTED FROM MOTORS' if manual else ('FRESH' if d['perception_fresh'] else 'STALE / STOP')
        self.pipeline.setText(ko(f'{self.stream.status} | {perception} | {d["stop_reason"]} | {restriction} | {d["notice"]}'))
        self.distance.setText(ko(f'Relative height target: {d["desired_height_ratio"]:.2f} (uncalibrated)'))
        self.camera_status.setText(ko(f'People {d["detected_people"]} | FPS {text(d["fps"],1)} | Processing {text(d["processing_ms"],1)} ms | Frame age {text(d["frame_age_s"])} s'))
        if result and result['completed_at']!=self.last_image_at:
            self.camera.set_frame(result,d['target_id']);self.last_image_at=result['completed_at']
        def age(timestamp):return max(0,now-timestamp) if timestamp is not None else None
        self.tracking.update_values({'People':d['detected_people'],'Target ID':d['target_id'],'Confirmed / Lost':f'{d["target_confirmed"]} / {d["target_lost"]}',
            'Confidence':d['detection_confidence'],'Center (px)':d['target_center'],'Last seen / age (s)':f'{text(d["target_last_seen"])} / {text(age(d["target_last_seen"]))}',
            'Horizontal error':d['horizontal_error'],'Relative height':d['relative_size']})
        raw_labels='\n'.join(f'{ko(r["name"])} / 신뢰도 {r["confidence"]:.2f} / 연결 목표 {text(r["associated_target"])}' for r in v['raw'])
        self.raw.update_values({'Hands':v['hand_count'],'Raw labels / confidence / owner':raw_labels or '없음','Sample age (s)':age(v.get('recognized_at'))})
        self.confirmed.update_values({'Current confirmation':g.get('confirmed') or 'NOT CONFIRMED',
            'Time / required (s)':f'{g.get("elapsed_s",0):.2f} / {g.get("required_s",0):.2f}',
            'Frames / required':f'{g.get("frames",0)} / {g.get("required_frames",0)}','Confidence threshold':self.engine.cfg['gesture']['confidence']})
        self.progress.setValue(round(g.get('progress',0)*100))
        self.applied.update_values({'Applied this update':g.get('applied_action') or 'NONE',
            'Last gesture action / age (s)':f'{text(g.get("last_applied_action"))} / {text(age(g.get("last_applied_at")))}',
            'Rejection / hold reason':g.get('rejection') or 'ACTION APPLIED'})
        def cmd(command):
            return 'UNKNOWN' if command is None else f'{command["vx_cm_s"]:+.2f} / {command["vy_cm_s"]:+.2f} cm/s / {command["w_rad_s"]:+.3f} rad/s'
        self.safe.update_values({'State':d['state'],'Requested vx / vy / w':cmd(d['requested']),'Safe vx / vy / w':cmd(d['safe']),
            'Limits or stop applied':d['limits_or_stop_applied'],'Final stop / status':d['stop_reason'],'Radar':'UNKNOWN / NOT CONNECTED'})
        self.arrow.set_data(d)
        safe_command=d['safe']
        prefix='MANUAL SLOW' if manual else 'SIMULATION SAFE'
        self.statusBar().showMessage(ko(f'{prefix}  vx {safe_command["vx_cm_s"]:+.2f} cm/s | vy {safe_command["vy_cm_s"]:+.2f} cm/s | w {safe_command["w_rad_s"]:+.3f} rad/s   /   {d["stop_reason"]}'))
        td=d['telemetry']
        reported={key:td.get(key) for key in ('cmd_vx','cmd_vy','cmd_w')} if td else None
        self.telemetry.update_values({'Connection':'CONNECTED / TELEMETRY VALID' if tr['connected'] else 'DISCONNECTED / NO VALID TELEMETRY',
            'Receive validity / reason':f'{tr.get("telemetry_valid")} / {tr.get("telemetry_reason","NO_TELEMETRY")}',
            'Last RX / age (s)':f'{text(tr.get("last_rx_at"))} / {text(tr.get("telemetry_age_s"))}',
            'Session / telemetry last_seq':f'{text(tr.get("session_id"))} / {text(td.get("last_seq") if td else None)}',
            'Last successful UDP send':tr.get('last_tx'),'Telemetry seq match (not ACK)':tr.get('processed_command',tr.get('acknowledged_command')),
            'ESP32 state / fault':f'{text(td.get("state") if td else None)} / {text(td.get("fault") if td else None)}',
            'ESP32 reported desired cmd':reported,'Post-limit body command':'NOT REPORTED BY CURRENT FIRMWARE',
            'Stop transmission / feedback':f'stop seq {text(tr.get("stop_seq"))} / {"VERIFIED BY FRESH TELEMETRY" if tr.get("stop_verified") else "NOT VERIFIED"} | TX error {text(tr.get("tx_error"))}'})
        motors=d['motors']
        for i in range(3):
            def at(key):return motors[key][i] if motors[key] is not None else None
            values=[f'M{i+1}',text(at('target_cm_s')),text(at('measured_cm_s')),text(at('target_rpm')),text(at('measured_rpm')),f'{text(at("pwm"))} / {text(at("encoder_count"))}']
            for j,value in enumerate(values):self.motor_table.setItem(i,j,QTableWidgetItem(ko(value)))
        self.graph.observe(d)
        self.update_motor_test(d)
        self.update_connection_diagnostics(d)
        if self.tabs.currentIndex()==1:
            self.vision_text.setPlainText('영상 인식 원문 기록: 출처·최신 여부·목표 ID·손동작·요청 및 안전 속도입니다.\n화면의 손동작 설명은 종합 현황에서 확인하세요. 아래 JSON 식별자와 모델 분류명은 원문을 유지합니다.\n'+json.dumps({key:d[key] for key in ['evidence_source','perception_fresh','target_id','vision','gesture','frame_at','vision_completed_at','requested','safe']},ensure_ascii=False,indent=2))

    def update_motor_test(self,d):
        tr=d['transport'];motor=d.get('motor_test')
        manual=d.get('control_source')=='MANUAL_MOTOR_TEST'
        trial=motor.get('trial') if motor else None
        history=motor.get('history',[]) if motor else []
        last=trial or (history[-1] if history else None)
        proof=manual and tr.get('stop_verified',False) and not trial
        complete=all(d['motors'][key] is not None for key in ('target_cm_s','measured_cm_s','pwm','encoder_count'))
        self.manual_arm.setEnabled(bool(proof and complete and self.lifted.isChecked() and self.burst_approval.isChecked()))
        self.heartbeat_button.setEnabled(bool(proof))
        ready=proof and bool(tr.get('manual_enabled'))
        for button in self.test_buttons.values():button.setEnabled(bool(ready))
        for i in range(3):
            m=d['motors']
            def at(key):return m[key][i] if m[key] is not None else None
            evidence=last['motors'][i]['encoder_evidence'] if last else 'NOT TESTED'
            values=[f'M{i+1}',text(at('target_cm_s')),text(at('measured_cm_s')),text(at('measured_rpm')),
                    text(at('pwm')),text(at('encoder_count')),evidence.replace('_',' ')]
            for j,value in enumerate(values):
                item=QTableWidgetItem(ko(value));item.setToolTip(ko(value));self.bench_motors.setItem(i,j,item)
        self.bench_motors.resizeRowsToContents()
        remaining=motor['remaining_s'] if motor else None
        self.burst_progress.setValue(round(100*remaining/trial['duration_s']) if trial and motor['status']=='RUNNING' else 0)
        command=motor['command'] if motor else None
        processing=('TELEMETRY SESSION / SEQ / STATE / CMD MATCH' if last['command_processed'] else 'NOT OBSERVED') if last else 'NOT TESTED'
        self.bench_state.update_values({'Source':motor['evidence_source'] if motor else 'NO MOTOR TELEMETRY / NOT TESTED',
            'Status / reason':f'{motor["status"] if motor else "DISABLED"} / {tr["reason"]}',
            'Remaining (s)':remaining,'Manual command vx / vy / w':command,'Command processing evidence':processing,
            'Stop feedback':'FRESH STOPPED TELEMETRY VERIFIED' if tr.get('stop_verified') else 'NOT VERIFIED',
            'Current / last trial':f'{last["session_id"]}:{last["id"]} {last["direction"]}' if last else 'NOT TESTED'})
        ids=[(t['session_id'],t['id'],t['physical_result']) for t in history]
        if ids!=getattr(self,'_trial_ids',None):
            selected_id=self.trial_selection.currentData()
            self._trial_ids=ids;self.trial_selection.clear()
            for t in history:self.trial_selection.addItem(f'세션 {t["session_id"]} / 시험 {t["id"]}: {ko(t["direction"])} / {ko(t["physical_result"])}',t['id'])
            if history:
                index=next((i for i,t in enumerate(history) if t['id']==selected_id),len(history)-1)
                self.trial_selection.setCurrentIndex(index)
        selected=next((t for t in history if t['id']==self.trial_selection.currentData()),None) or trial
        report=('시험 원문: 명령 처리·엔코더 관측·사용자 육안 관찰은 별개입니다. JSON 필드명은 원문입니다.\n'+json.dumps(selected,ensure_ascii=False,indent=2)) if selected else ko('No motor trial performed. Actual hardware: NOT TESTED.')
        if self.trial_text.toPlainText()!=report:self.trial_text.setPlainText(report)

    def update_connection_diagnostics(self,d):
        host=self.engine.client.host if self.engine.client else self.host.text().strip()
        motor=d.get('motor_test') or {};trial=motor.get('trial') or (motor.get('history') or [None])[-1]
        network=self.network_probe.result
        if network and network['target']!=host:network=None
        attempt=self.engine.connection_attempt
        if attempt and attempt.get('host')!=host:attempt=None
        report=diagnostic_report(host,d['transport'],network,attempt,trial)
        self.last_diagnostic_report=report
        names=['① ESP32 IP 설정','② UDP 수신 대기','③ ESP32 응답','④ 세션·생존 확인','⑤ ESP32 제어 상태','⑥ 모터 시험 준비']
        status={'OK':'정상','FAIL':'실패','PENDING':'확인 중','NOT_TESTED':'미검증'}
        for i,step in enumerate(report['steps']):
            for j,value in enumerate((names[i],status[step['status']],step['message'])):
                self.diagnostic_steps.setItem(i,j,QTableWidgetItem(value))
        self.diagnostic_steps.resizeRowsToContents()
        self.network_check.setEnabled(not self.network_probe.running)
        n=network
        if self.network_probe.running:self.network_status.setText('네트워크와 수신 포트를 무구동 점검 중입니다. 화면 조작과 정지 감시는 계속 동작합니다.')
        elif self.network_probe.error:self.network_status.setText('네트워크 점검 오류: '+self.network_probe.error+' · 현재 환경의 권한과 네트워크를 확인하세요.')
        elif n:
            same={True:'같은 직접 연결 대역',False:'다른 대역 (라우팅 여부 추가 확인 필요)',None:'대역 미확인'}[n['same_subnet']]
            ping={True:'응답 있음',False:'응답 없음 — 이것만으로 UDP 실패를 확정하지 않음',None:'미검증'}[n['ping_reply']]
            active=next((p.get('NetworkCategory') for p in n.get('connection_profiles',[]) if p.get('InterfaceAlias')==n.get('selected_interface')),None)
            category={'Public':'공용','Private':'개인','DomainAuthenticated':'도메인',0:'공용',1:'개인',2:'도메인'}.get(active,'미확인')
            enabled=[{'Domain':'도메인','Private':'개인','Public':'공용'}.get(p.get('Name'),str(p.get('Name'))) for p in n.get('firewall_profiles',[]) if str(p.get('Enabled')).lower() in ('true','1')]
            self.network_status.setText(f'점검 시각 {n["checked_at"]} | 노트북 선택 IP {n["selected_local_ip"]} | 대상 {n["target"]}\n{same} | ICMP {ping} | 수신 포트 {text(n["port_available"])}\n현재 연결 범주: {category} | 방화벽 사용 프로필: {", ".join(enabled) or "미확인"}\n포트 점유 PID와 프로필 원문은 아래에서 확인하세요. 방화벽의 UDP 차단 여부와 공유기·핫스팟 단말 격리는 추가 확인이 필요합니다.')
        else:self.network_status.setText('현재 입력 주소에 대한 네트워크 점검은 미검증입니다. 네트워크 변경 후 다시 점검하세요.')
        t=d['transport']
        details=dict(설명='송신 성공·수신 검증·실제 회전은 별개이며 JSON 필드명은 원문입니다.',
            시각기준='송수신 시각은 실행 내 경과 비교용 단조 시계(초)이며 JSONL 각 행에는 실제 시각이 별도로 기록됩니다.',
            송신=dict(시각=t.get('last_tx_at'),목적지=t.get('tx_destination'),JSON=t.get('last_tx'),소켓오류=t.get('tx_error_details') or t.get('tx_error')),
            수신=dict(시각=t.get('last_rx_any_at'),송신자=t.get('last_rx_source'),JSON=t.get('last_rx_json'),검증사유=ko(t.get('validation_detail','NO_TELEMETRY')),거부집계=t.get('rx_rejections')),
            네트워크원문=network,
            통신감시중단=t.get('guard_error'),
            원인후보=report['findings'])
        value=json.dumps(details,ensure_ascii=False,indent=2)
        if self.communication_text.toPlainText()!=value:self.communication_text.setPlainText(value)

    def apply_style(self):
        self.setStyleSheet('''
          QMainWindow,QWidget { background:#111923; color:#dfE7f0; font-family:Malgun Gothic; font-size:12px; }
          QLabel#title { font-size:17px; font-weight:700; letter-spacing:1px; }
          QLabel#mode { background:#654719; color:#ffda91; padding:8px; border-radius:5px; font-weight:700; }
          QLabel#mode[hardware="true"] { background:#772c35; color:#ffe0e2; }
          QGroupBox { background:#1a2533; border:1px solid #2d3d50; border-radius:6px; margin-top:12px; padding:12px 8px 6px; font-weight:600; }
          QGroupBox::title { subcontrol-origin:margin; left:10px; color:#83b8f9; }
          QGroupBox QLabel { background:transparent; font-weight:400; }
          QPushButton { background:#293e55; border:1px solid #3a5571; padding:9px 12px; border-radius:4px; }
          QPushButton:hover { background:#365573; }
          QPushButton:disabled { background:#1a2634; border-color:#293849; color:#66788e; }
          QPushButton#estop { background:#ba3e48; border:1px solid #df626c; color:white; font-weight:700; }
          QTabWidget::pane { border:1px solid #2d3d50; }
          QTabBar::tab { padding:10px 22px; background:#172332; }
          QTabBar::tab:selected { color:#9dcaff; background:#26394f; }
          QLineEdit,QPlainTextEdit,QComboBox,QTableWidget { background:#101923; border:1px solid #34465d; padding:6px; }
          QHeaderView::section { background:#26394f; color:#dfe7f0; padding:8px; border:0; }
          QProgressBar { border:1px solid #34465d; text-align:center; height:18px; }
          QProgressBar::chunk { background:#3f86bd; }
        ''')

    def closeEvent(self,event):
        if not self.closing:
            self.closing=True;self.timer.stop();self.engine.request('ESTOP')
            QApplication.instance().removeEventFilter(self)
            try:self.engine.close()
            except Exception as exc:self.shutdown_error=str(exc)
            finally:self.stream.close()
        event.accept()


def run_dashboard(config,hardware=False,preview=False,smoke_seconds=None):
    app=QApplication.instance() or QApplication([])
    window=Dashboard(config,hardware,preview)
    window.show()
    if smoke_seconds is not None:QTimer.singleShot(int(smoke_seconds*1000),window.close)
    previous=signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT,lambda signum,frame:window.close())
    try:result=app.exec()
    finally:
        window.close()
        signal.signal(signal.SIGINT,previous)
    return 1 if getattr(window,'shutdown_error',None) else result
