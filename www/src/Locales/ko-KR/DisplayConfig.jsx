export default {
	'header-text': '디스플레이 설정',
	'sub-header-text':
		'모노크롬 디스플레이 사용하여 컨트롤러의 상태와 버튼 작동 상태를 표시할 수 있습니다. 사용 중인 디스플레이가 다음 요구 사항을 충족하는지 확인하세요.',
	'list-text':
		'<1>128x64해상도의 단색 디스플레이</1> <1>I2C 연결을 사용하는 SSD1306, SH1106, SH1107 또는 호환 디스플레이</1> <1>3.3v 지원</1>',
	section: {
		'hardware-header': '하드웨어 옵션',
		'screen-header': '스크린 옵션',
		'layout-header': '레이아웃 옵션',
		'mode-header': '모드 옵션',
		'button-layout-header': '버튼 레이아웃',
		'status-layout-header': '상태 바 레이아웃',
		'history-layout-header': '입력 기록 레이아웃',
	},
	table: {
		header:
			'다음 표를 참조하여 설정된 SDA 및 SCL 핀을 기반으로 사용할 I2C 블록을 결정합니다:',
		'sda-scl-pins-header': 'SDA/SCL 핀',
		'i2c-block-header': 'I2C 블록',
	},
	form: {
		'i2c-block-label': 'I2C 블록',
		'sda-pin-label': 'SDA 핀',
		'scl-pin-label': 'SCL 핀',
		'i2c-address-label': 'I2C 주소',
		'i2c-speed-label': 'I2C 속도',
		'flip-display-label': '표시 반전',
		'invert-display-label': '흑백 반전',
		'button-layout-label': '버튼 레이아웃 (좌)',
		'button-layout-right-label': '버튼 레이아웃 (우)',
		'button-layout-orientation': '버튼 레이아웃 방향',
		'button-layout-custom-header': '커스텀 버튼 레이아웃 매개변수',
		'button-layout-custom-left-label': '좌측 레이아웃',
		'button-layout-custom-right-label': '우측 레이아웃',
		'button-layout-custom-start-x-label': 'X좌표 시작 위치',
		'button-layout-custom-start-y-label': 'Y좌표 시작 위치',
		'button-layout-custom-button-radius-label': '버튼 반경',
		'button-layout-custom-button-padding-label': '버튼 간격',
		'splash-mode-label': '스플래시 모드',
		'splash-duration-label': '스플래시 표시 시간 (초, 항상표시는 0)',
		'display-saver-timeout-label': '화면 보호기 동작 시간 (분, 꺼짐은 0)',
		'screen-saver-mode-label': '화면 보호기 모드',
		'inverted-label': '반전',
		'power-management-header': '전원 관리',
		'turn-off-when-suspended': '대기모드에서 끄기',
		'input-history-label': '입력 기록',
		'display-state': {
			disabled: '사용안함',
			enabled: '사용',
		},
		'flip-display': {
			none: '안함',
			flip: '상하 반전',
			mirror: '좌우 반전',
			'flip-mirror': '상하 및 좌우 반전',
		},
		'splash-modes': {
			enabled: '사용 (사용자 스플래시 화면)',
			'close-in': '로고 클로즈인',
			'close-in-custom': '사용자 로고 클로즈인',
			disabled: '사용안함',
		},
		'saver-modes': {
			'display-off': '디스플레이 끄기',
			snow: '눈',
			bounce: '로고 움직임',
			pipes: '파이프',
			toast: '토스트',
		},
		'layout-modes': {
			standard: '기본',
			southpaw: '역배열',
			switched: '좌우 변경',
		},
		'status-header': {
			'input-mode': '입력 모드',
			'turbo-mode': '터보',
			'dpad-mode': 'D-패드 모드',
			'socd-mode': 'SOCD 모드',
			'macro-mode': '매크로',
			'profile-mode': '프로필',
		},
	},
};
