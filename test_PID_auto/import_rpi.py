"""
Chargement et inférence du modèle IA quantifié
Compatible PC (x86) et Raspberry Pi (ARM)
"""
import torch
import numpy as np
import time
import platform

print("=" * 60)
print("CHARGEMENT MODÈLE IA - PC & Raspberry Pi")
print("=" * 60)

# 1. Détection automatique de la plateforme
print(f"\nPlateforme détectée: {platform.machine()}")
if 'arm' in platform.machine().lower() or 'aarch64' in platform.machine().lower():
    torch.backends.quantized.engine = 'qnnpack'
    print("Moteur: QNNPACK (ARM/Raspberry Pi)")
else:
    try:
        torch.backends.quantized.engine = 'fbgemm'
        print("Moteur: FBGEMM (x86/PC)")
    except:
        print("Moteur: Par défaut")

# 2. Chargement du modèle
MODEL_PATH = "drive_assist_rpi_int8.pt"
print(f"\nChargement du modèle: {MODEL_PATH}")
try:
    rpi_model = torch.jit.load(MODEL_PATH)
    rpi_model.eval()
    print("✓ Modèle chargé avec succès")
except Exception as e:
    print(f"✗ Erreur chargement: {e}")
    print("Assurez-vous d'avoir exécuté export_rpi.py d'abord")
    exit(1)

# 3. Fonction d'inférence
# State: [yaw_err, yaw_rate, ax, ay, az, motor_L, motor_R, dir_y, dir_x, ramp_speed]
def get_action(state_10d):
    """
    Inférence rapide du modèle IA
    
    Args:
        state_10d: liste/array de 10 valeurs normalisées:
            - yaw_error / 45.0 (clipped -1,1)
            - yaw_rate / 100.0 (clipped -1,1)
            - ax / 32768.0 (clipped -1,1)
            - ay / 32768.0 (clipped -1,1)
            - az / 32768.0 (clipped -1,1)
            - motor_L / 255.0 (clipped -1,1)
            - motor_R / 255.0 (clipped -1,1)
            - dir_y (clipped -1,1)
            - dir_x (clipped -1,1)
            - ramp_speed / 200.0 (clipped 0,1)
    
    Returns:
        action: [trim_L, trim_R, ramp_boost] chacun dans [-1, 1]
    """
    start_time = time.perf_counter()
    
    state_np = np.array(state_10d, dtype=np.float32)
    if len(state_np) != 10:
        raise ValueError(f"State doit avoir 10 dimensions, reçu: {len(state_np)}")
    
    state_tensor = torch.from_numpy(state_np).unsqueeze(0)
    
    with torch.no_grad():
        action = rpi_model(state_tensor).numpy()[0]
    
    calc_time = (time.perf_counter() - start_time) * 1000
    
    return action, calc_time

# 4. Conversion vers valeurs physiques
def decode_action(action):
    """Convertit l'action [-1,1] en valeurs physiques"""
    max_trim = 15.0
    max_ramp_boost = 0.5
    
    trim_L = action[0] * max_trim
    trim_R = action[1] * max_trim
    ramp_boost = action[2] * max_ramp_boost
    
    return trim_L, trim_R, ramp_boost

# --- TEST ---
if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("TEST D'INFÉRENCE")
    print("=" * 60)
    
    # Test 1: Robot va droit, légère dérive
    print("\nTest 1: Dérive yaw de 10°")
    state = [0.22, 0.1, 0, 0, 0.5, 0.5, 0.5, 1.0, 0, 0.4]  # yaw_err/45=0.22
    action, latency = get_action(state)
    trim_L, trim_R, ramp = decode_action(action)
    print(f"  Action: trim_L={trim_L:+.1f}, trim_R={trim_R:+.1f}, ramp={ramp:+.2f}")
    print(f"  Latence: {latency:.2f} ms")
    
    # Test 2: Robot tourne
    print("\nTest 2: Virage à gauche")
    state = [0, 0.5, 0, 0, 0.8, 0.3, 0.6, 0.5, -0.5, 0.4]
    action, latency = get_action(state)
    trim_L, trim_R, ramp = decode_action(action)
    print(f"  Action: trim_L={trim_L:+.1f}, trim_R={trim_R:+.1f}, ramp={ramp:+.2f}")
    print(f"  Latence: {latency:.2f} ms")
    
    # Benchmark
    print("\nBenchmark (100 inférences)...")
    times = []
    for _ in range(100):
        _, t = get_action([0.1, 0.05, 0, 0, 0.5, 0.5, 0.5, 1.0, 0, 0.4])
        times.append(t)
    
    print(f"  Moyenne: {np.mean(times):.2f} ms")
    print(f"  Min: {np.min(times):.2f} ms")
    print(f"  Max: {np.max(times):.2f} ms")
    
    print("\n" + "=" * 60)
    print("✅ Test terminé avec succès")
    print("=" * 60)