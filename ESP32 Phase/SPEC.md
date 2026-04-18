# CSI + YOLO Human Detection & Tracking System — Specification

## System Overview
A WiFi CSI-based human detection and tracking system with optional multi-ESP32 support, fused with YOLO computer vision from a Logitech Brio 105 webcam. Handles unsynchronized ESP32 phase coherence via 100ms phase smoothing. Supports MUSIC AoA (with external antenna array) and FFT-based trilateration/triangulation with method overlap.

## Hardware
- **Camera**: Logitech Brio 105 (USB webcam, 1080p30)
- **Primary WiFi**: ESP32 (single CSI source)
- **Multi-ESP Extension**: 2+ ESP32s for AoA/trilateration (phase incoherent)

## Key Research-Informed Design Decisions
1. **Single ESP32 mode**: Amplitude-based presence detection + YOLO fusion
2. **Multi-ESP32 mode**: FFT-based ToF trilateration (primary) + MUSIC AoA (optional, needs antenna array) + YOLO fusion
3. **100ms phase smoothing**: Savitzky-Golay moving window — reduces PDD variance, tracks slow phase drift
4. **Sensor fusion**: EKF with adaptive covariance — camera corrects WiFi drift during visible periods
5. **MUSIC limitation**: ESP32 single antenna cannot run MUSIC directly; requires external antenna switch or phase-synced array

---

## Module Architecture

```
csi-tracking-system/
├── src/
│   ├── __init__.py
│   ├── csi_core.py           # CSI data acquisition, parsing, filtering
│   ├── music_aoa.py          # MUSIC algorithm for AoA estimation
│   ├── multi_esp_fft.py      # Multi-ESP FFT & 100ms phase smoothing
│   ├── localization.py       # Triangulation + trilateration + overlap
│   ├── yolo_tracker.py       # YOLO detection & tracking on Brio 105
│   ├── sensor_fusion.py      # EKF-based WiFi + Camera fusion
│   ├── pipeline.py           # Real-time processing pipeline
│   └── config.py             # Configuration management
├── firmware/
│   └── esp32_csi_tx/         # ESP32 Arduino/PlatformIO firmware
│       ├── esp32_csi_tx.ino
│       └── config.h
├── main.py                   # CLI entry point (--useMultiESP flag)
├── requirements.txt
├── README.md
└── setup.py
```

---

## Module Specifications

### 1. `config.py` — Configuration Management

```python
@dataclass
class CSIConfig:
    esp32_ip: List[str]           # ESP32 IP addresses (UDP mode)
    esp32_port: int = 5005        # UDP port for CSI streaming
    sample_rate: int = 100        # Target CSI sample rate (Hz)
    window_size_ms: int = 100     # Phase smoothing window (100ms)
    subcarrier_count: int = 52    # ESP32 HT20 subcarriers
    use_multi_esp: bool = False   # --useMultiESP flag
    
@dataclass  
class YOLOConfig:
    model: str = "yolo11n.pt"     # Ultralytics model
    conf_thresh: float = 0.5      # Detection confidence threshold
    classes: List[int] = [0]      # Person class only
    track_buffer: int = 30        # ByteTrack buffer
    camera_id: int = 0            # USB camera index
    resolution: Tuple[int,int] = (640, 480)
    
@dataclass
class FusionConfig:
    process_noise: float = 0.1    # EKF Q
    measurement_noise_wifi: float = 2.0   # EKF R for WiFi
    measurement_noise_cam: float = 0.3    # EKF R for camera
    fusion_weight_wifi: float = 0.3       # Static weight fallback
    fusion_weight_cam: float = 0.7
```

### 2. `csi_core.py` — CSI Data Acquisition & Preprocessing

**Responsibilities**:
- UDP socket listener for ESP32 CSI packets
- Binary CSI packet parsing (ESP32 CSI frame format)
- Phase unwrapping (2π discontinuity resolution)
- Hampel filter for outlier removal per subcarrier
- Phase sanitization (linear fitting + subtraction across subcarriers)
- Amplitude normalization
- Sliding window buffer management

**Key Functions**:
```python
def parse_csi_frame(data: bytes) -> Dict:
    """Parse ESP32 CSI binary frame.
    Returns: {'timestamp': float, 'rssi': int, 'csi_bytes': bytes,
              'amplitude': np.ndarray, 'phase': np.ndarray}
    """
    
def hampel_filter(data: np.ndarray, window: int = 5, n_sigma: float = 3.0) -> np.ndarray:
    """Hampel outlier detection per subcarrier."""
    
def phase_sanitize(phase: np.ndarray) -> np.ndarray:
    """Linear fit across subcarriers, remove slope+offset (SFO/CFO residual)."""
    
def unwrap_phase(phase: np.ndarray) -> np.ndarray:
    """2π phase unwrapping along subcarrier dimension."""
    
def normalize_amplitude(amp: np.ndarray) -> np.ndarray:
    """Per-window amplitude normalization."""
    
class CSISource:
    """Threaded CSI data source from UDP stream."""
    def __init__(self, ip: str, port: int);
    def start(self);
    def get_window(self, n_samples: int) -> np.ndarray;
    def stop(self);
```

### 3. `music_aoa.py` — MUSIC Algorithm

**Responsibilities**:
- Covariance matrix estimation from spatial samples
- Eigendecomposition for signal/noise subspace separation
- 2D MUSIC pseudospectrum (AoA vs ToF) when antenna array available
- Peak detection for angle estimation
- Support virtual arrays via subcarrier grouping (SpotFi-style)

**Key Functions**:
```python
def estimate_covariance(X: np.ndarray) -> np.ndarray:
    """Sample covariance matrix from M x N snapshots."""
    
def music_spectrum(R: np.ndarray, steering_vectors: np.ndarray, n_signals: int) -> np.ndarray:
    """Compute MUSIC pseudospectrum.
    Args: R=covariance, steering_vectors=M x P grid, n_signals
    Returns: pseudospectrum over angle grid
    """
    
def spatial_smoothing(X: np.ndarray, subarray_size: int) -> np.ndarray:
    """Forward-backward spatial smoothing for coherent signals."""
    
def build_steering_ula(angles: np.ndarray, d: float, wavelength: float, M: int) -> np.ndarray:
    """Build ULA steering vectors for angle grid."""
    
def estimate_aoa(csi_data: np.ndarray, n_signals: int = 3, 
                  antenna_positions: Optional[np.ndarray] = None) -> List[float]:
    """Main AoA estimation entry point.
    Uses virtual subcarrier array if single antenna,
    or physical array if antenna_positions provided.
    """
```

### 4. `multi_esp_fft.py` — Multi-ESP FFT & Phase Smoothing

**Responsibilities**:
- 100ms sliding window phase smoothing per ESP32
- FFT per ESP32 for channel state waveform analysis
- Phase unwrapping and drift compensation
- Time-offset estimation between ESP32s
- Distance estimation from phase slope
- Produce smoothed CSI frames aligned in time

**Key Functions**:
```python
def phase_smooth_savgol(phase: np.ndarray, window_ms: int = 100, 
                        fs: int = 100, polyorder: int = 3) -> np.ndarray:
    """Savitzky-Golay phase smoothing over 100ms window."""
    
def estimate_phase_slope(phase: np.ndarray, subcarrier_frequencies: np.ndarray) -> float:
    """Linear regression of phase vs subcarrier frequency → slope ∝ ToF."""
    
def csi_to_tof(csi_frame: np.ndarray, bw_hz: float = 20e6) -> float:
    """Convert CSI phase slope to time-of-flight estimate."""
    
def csi_to_distance(csi_frame: np.ndarray, bw_hz: float = 20e6) -> float:
    """Convert ToF to distance (d = c * τ / 2 for round-trip)."""
    
def fft_smooth_csi(csi_frames: List[np.ndarray], n_fft: int = 256) -> np.ndarray:
    """Apply FFT-based smoothing to CSI amplitude/phase series."""
    
def align_esp_timestamps(esp_streams: Dict[str, List[Dict]]) -> Dict[str, List[Dict]]:
    """Time-align multiple ESP32 streams using nearest-neighbor matching."""
    
def compensate_cfo_drift(phase_series: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    """Estimate and remove linear CFO drift from phase time series."""
    
class MultiESPProcessor:
    """Orchestrates multi-ESP32 CSI processing with 100ms smoothing."""
    def __init__(self, esp_configs: List[CSIConfig], window_ms: int = 100);
    def process_frame(self, esp_id: str, frame: Dict) -> Optional[Dict];
    def get_smoothed_distances(self) -> Dict[str, float];
    def get_sync_status(self) -> Dict;
```

### 5. `localization.py` — Triangulation + Trilateration + Overlap

**Responsibilities**:
- Trilateration: weighted least squares from multiple distance estimates
- Triangulation: angle intersection from multiple AoA estimates
- Overlap method: consistency checking between triangulation and trilateration
- GDOP calculation for anchor geometry assessment
- RANSAC for outlier rejection
- EKF-based position tracking

**Key Functions**:
```python
def trilaterate_lsq(distances: np.ndarray, anchors: np.ndarray, 
                    weights: Optional[np.ndarray] = None) -> np.ndarray:
    """Weighted least squares trilateration.
    Args: distances=N, anchors=N x 2 (x,y), weights=N
    Returns: estimated position (x, y)
    """
    
def triangulate_angles(angles: np.ndarray, anchor_positions: np.ndarray,
                       anchor_orientations: np.ndarray) -> np.ndarray:
    """Triangulate from AoA measurements at multiple anchors.
    Args: angles=N (radians), positions=N x 2, orientations=N
    Returns: estimated position (x, y)
    """
    
def overlap_consistency_check(pos_tri: np.ndarray, pos_tri_lat: np.ndarray,
                               cov_tri: np.ndarray, cov_tri_lat: np.ndarray) -> Tuple[np.ndarray, float]:
    """Check consistency between triangulation and trilateration.
    Returns: fused_position, confidence_score
    """
    
def compute_gdop(anchor_positions: np.ndarray, target: np.ndarray) -> float:
    """Geometric Dilution of Precision."""
    
def ransac_localization(measurements: List[Dict], threshold: float = 0.5) -> np.ndarray:
    """RANSAC for robust localization with outliers."""
    
class LocalizationEngine:
    """Fuses trilateration and triangulation with overlap checking."""
    def __init__(self, anchors: List[Dict]);
    def update_distances(self, esp_id: str, distance: float, variance: float);
    def update_angles(self, esp_id: str, angle: float, variance: float);
    def estimate_position(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns (position, covariance) or None if insufficient data."""
```

### 6. `yolo_tracker.py` — YOLO Detection & Tracking

**Responsibilities**:
- OpenCV video capture from Brio 105
- YOLO11 inference via Ultralytics
- ByteTrack person tracking across frames
- Bottom-center pixel extraction for position mapping
- JSON output format for fusion module

**Key Functions**:
```python
def initialize_camera(camera_id: int = 0, resolution: Tuple[int,int] = (640,480)) -> cv2.VideoCapture:
    """Initialize Logitech Brio 105 with optimal settings."""
    
def run_yolo_detection(frame: np.ndarray, model: YOLO, conf: float = 0.5) -> List[Dict]:
    """Run YOLO inference, return person detections.
    Returns: [{'bbox': [x1,y1,x2,y2], 'confidence': float, 
               'bottom_center': [x, y], 'track_id': int}]
    """
    
def pixel_to_world(pixel: np.ndarray, homography: Optional[np.ndarray] = None) -> np.ndarray:
    """Convert pixel coordinates to world coordinates using homography."""
    
class YOLOTracker:
    """YOLO + ByteTrack person tracker."""
    def __init__(self, config: YOLOConfig);
    def start(self);
    def get_tracks(self) -> List[Dict];
    def get_frame_with_tracks(self) -> np.ndarray;
    def stop(self);
    def get_homography_matrix(self, pixel_points: np.ndarray, world_points: np.ndarray) -> np.ndarray:
        """Calibrate pixel-to-world mapping."""
```

### 7. `sensor_fusion.py` — EKF-Based WiFi + Camera Fusion

**Responsibilities**:
- Extended Kalman Filter for position tracking
- Dynamic weight assignment based on measurement confidence
- Camera-assisted phase drift correction (novel)
- Sensor dropout handling with covariance inflation
- Consistency checking (overlap method integration)

**Key Functions**:
```python
class CSI_YOLO_FusionEKF:
    """EKF fusing WiFi CSI localization with YOLO camera tracking."""
    
    def __init__(self, Q: float = 0.1, R_wifi: float = 2.0, R_cam: float = 0.3):
        """Initialize EKF with process and measurement noise."""
        self.x = np.zeros(4)   # [px, py, vx, vy]
        self.P = np.eye(4) * 10
        self.Q = np.eye(4) * Q
        self.R_wifi = np.eye(2) * R_wifi
        self.R_cam = np.eye(2) * R_cam
        
    def predict(self, dt: float = 0.1):
        """Constant velocity prediction model."""
        F = np.array([[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        
    def update_wifi(self, pos: np.ndarray, variance: Optional[float] = None):
        """Update with WiFi-derived position estimate."""
        
    def update_camera(self, pos: np.ndarray, variance: Optional[float] = None):
        """Update with camera-derived position estimate."""
        
    def get_position(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns (position, covariance)."""
        return self.x[:2], self.P[:2, :2]
        
    def estimate_wifi_bias(self, camera_pos: np.ndarray, wifi_pos: np.ndarray) -> np.ndarray:
        """Estimate WiFi measurement bias when camera is available.
        This enables camera-assisted phase calibration."""

def adaptive_weight_assignment(wifi_variance: float, cam_variance: float,
                               wifi_snr: float, cam_confidence: float) -> Tuple[float, float]:
    """Dynamic weight assignment based on measurement quality."""
```

### 8. `pipeline.py` — Real-Time Processing Pipeline

**Responsibilities**:
- Multiprocessing pipeline with queues
- Producer: CSI acquisition threads (one per ESP32)
- Producer: YOLO tracking thread
- Consumer: Fusion EKF thread
- Visualization output

**Key Classes**:
```python
class TrackingPipeline:
    """Orchestrates real-time CSI + YOLO tracking pipeline."""
    def __init__(self, csi_config: CSIConfig, yolo_config: YOLOConfig, 
                 fusion_config: FusionConfig);
    def start(self);
    def stop(self);
    def get_state(self) -> Dict;
    
class PipelineVisualizer:
    """Real-time visualization of tracking state."""
    def __init__(self);
    def update(self, state: Dict);
    def render(self) -> np.ndarray;
```

### 9. `main.py` — CLI Entry Point

```python
import argparse

def main():
    parser = argparse.ArgumentParser(description='CSI + YOLO Human Tracking')
    parser.add_argument('--useMultiESP', action='store_true',
                       help='Enable multi-ESP32 mode with phase smoothing')
    parser.add_argument('--esp-ips', nargs='+', default=['192.168.1.100'],
                       help='ESP32 IP addresses')
    parser.add_argument('--port', type=int, default=5005)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--visualize', action='store_true')
    args = parser.parse_args()
    
    # Initialize pipeline
    # Start processing
    
if __name__ == '__main__':
    main()
```

### 10. ESP32 Firmware (`firmware/esp32_csi_tx/`)

```cpp
// Key configuration:
// - WiFi mode: STA connected to AP, or AP mode
// - CSI callback: print_csi_data() outputs binary frames
// - Packet rate: configurable (100-1000 Hz)
// - Output format: binary UDP frames
// - Includes: timestamp, RSSI, CSI bytes, MAC address

void print_csi_data(uint8_t* data, uint32_t len) {
    // Output binary frame: [magic(4)] [timestamp(8)] [rssi(4)] [csi_len(4)] [csi_data(N)]
}

void setup() {
    // WiFi init
    // CSI callback registration
    // UDP client init (if streaming over network)
}

void loop() {
    // Send ping packets to trigger CSI
    // or wait for broadcast packets
}
```

---

## Data Flow Diagram

```
[ESP32 #1] ──UDP──┐
[ESP32 #2] ──UDP──┼──→ csi_core.py ──→ multi_esp_fft.py ──→ localization.py ─┐
[ESP32 #N] ──UDP──┘    (parse/filter)    (100ms smooth)     (tri/trilat)      │
                                                                                ├─→ sensor_fusion.py ──→ Output
[Brio 105] ──USB──→ yolo_tracker.py ────────────────────────────────────────┘
                       (detect/track)                                         (EKF fusion)
```

## Dependencies (requirements.txt)
```
numpy>=1.24.0
scipy>=1.10.0
opencv-python>=4.8.0
ultralytics>=8.3.0
filterpy>=1.4.5
matplotlib>=3.7.0
pyyaml>=6.0
```
