# Migration Plan: MotorDigitizer to Main Library

## Overview

This document outlines the comprehensive plan to move the MotorDigitizer system from `examples/` into the main `mks_servo_can` library structure, making it a core library feature.

## 1. Library Structure Integration

### 1.1 Target Location

**Primary Location:** `mks_servo_can_library/mks_servo_can/digitizer/`

**Rationale:**
- Creates a dedicated namespace for digitizing functionality
- Follows library's modular organization pattern (like `kinematics/`)
- Allows for future expansion of digitizing capabilities
- Maintains clean separation from core motor control functionality

### 1.2 Proposed Directory Structure

```
mks_servo_can_library/mks_servo_can/
├── digitizer/
│   ├── __init__.py                    # Module exports
│   ├── base_digitizer.py             # Core MotorDigitizer class
│   ├── data_structures.py            # DigitizedSequence, DigitizedPoint, etc.
│   ├── precision_analyzer.py         # PlaybackStats and analysis functions  
│   ├── surface_mapping.py            # Enhanced height mapping functionality
│   └── utils.py                      # Utility functions (create_linear_axis, etc.)
├── __init__.py                       # Add digitizer exports
└── [existing modules...]
```

### 1.3 File Organization and Responsibilities

#### `digitizer/__init__.py`
```python
"""
Motor Digitizer Module

Provides recording, playback, and precision testing capabilities for MKS servo motors.
"""

from .base_digitizer import MotorDigitizer
from .data_structures import (
    DigitizedPoint,
    DigitizedSequence, 
    PlaybackStats
)
from .precision_analyzer import PrecisionAnalyzer
from .surface_mapping import (
    SurfacePoint,
    SurfaceMap, 
    EnhancedHeightMapGenerator
)
from .utils import create_linear_axis, create_rotary_axis

__all__ = [
    "MotorDigitizer",
    "DigitizedPoint", 
    "DigitizedSequence",
    "PlaybackStats",
    "PrecisionAnalyzer",
    "SurfacePoint",
    "SurfaceMap",
    "EnhancedHeightMapGenerator",
    "create_linear_axis",
    "create_rotary_axis"
]
```

#### `digitizer/base_digitizer.py`
- Core `MotorDigitizer` class
- Recording and playback engine
- Basic precision testing functionality
- Data persistence (save/load sequences)

#### `digitizer/data_structures.py`
- `DigitizedPoint` dataclass
- `DigitizedSequence` dataclass  
- `PlaybackStats` dataclass
- JSON serialization helpers

#### `digitizer/precision_analyzer.py`
- `PrecisionAnalyzer` class for statistical analysis
- Assessment functions (EXCELLENT/GOOD/FAIR/POOR)
- Repeatability analysis
- Path complexity calculations

#### `digitizer/surface_mapping.py`
- `SurfacePoint` and `SurfaceMap` dataclasses
- `EnhancedHeightMapGenerator` class
- Surface analysis and interpolation functions

#### `digitizer/utils.py`
- `create_linear_axis()` and `create_rotary_axis()` convenience functions
- Common digitizer utility functions
- Configuration helpers

## 2. Main Library Integration

### 2.1 Update `mks_servo_can/__init__.py`

Add digitizer exports to main library interface:

```python
# Add to imports section
from .digitizer import (
    MotorDigitizer,
    DigitizedPoint,
    DigitizedSequence,
    PlaybackStats,
    PrecisionAnalyzer,
    SurfacePoint,
    SurfaceMap,
    EnhancedHeightMapGenerator,
    create_linear_axis,
    create_rotary_axis
)

# Add to __all__ list
__all__ = [
    # ... existing exports ...
    
    # Digitizer classes
    "MotorDigitizer",
    "DigitizedPoint",
    "DigitizedSequence", 
    "PlaybackStats",
    "PrecisionAnalyzer",
    "SurfacePoint",
    "SurfaceMap", 
    "EnhancedHeightMapGenerator",
    "create_linear_axis",
    "create_rotary_axis",
]
```

### 2.2 Version Bump

Update version in `__init__.py`:
```python
__version__ = "0.2.0"  # Minor version bump for new feature
```

### 2.3 Dependencies

Add optional dependencies to `setup.py`:
```python
install_requires=[
    # ... existing requirements ...
],
extras_require={
    'digitizer': ['numpy>=1.20.0'],  # For enhanced surface calculations
    'full': ['numpy>=1.20.0', 'matplotlib>=3.3.0'],  # For visualization
}
```

## 3. Test Suite Implementation

### 3.1 Test Structure

```
tests/
├── unit/
│   ├── test_digitizer_base.py           # Core MotorDigitizer tests
│   ├── test_digitizer_data_structures.py # Data structure tests
│   ├── test_precision_analyzer.py       # Analysis function tests
│   └── test_surface_mapping.py          # Surface mapping tests
├── integration/
│   ├── test_digitizer_with_simulator.py # End-to-end digitizer tests
│   └── test_surface_mapping_integration.py # Surface mapping integration
└── performance/
    └── test_digitizer_performance.py    # Performance benchmarks
```

### 3.2 Unit Tests Required

#### `test_digitizer_base.py`
```python
class TestMotorDigitizer:
    def test_init(self):
        """Test MotorDigitizer initialization"""
    
    def test_add_axis(self):
        """Test adding axes to digitizer"""
    
    def test_initialize_axes(self):
        """Test axis initialization"""
    
    def test_get_axis_configs(self):
        """Test axis configuration extraction"""
    
    @pytest.mark.asyncio
    async def test_recording_setup(self):
        """Test recording setup and teardown"""
    
    @pytest.mark.asyncio 
    async def test_save_load_sequence(self):
        """Test sequence persistence"""

class TestRecordingEngine:
    @pytest.mark.asyncio
    async def test_recording_loop(self):
        """Test recording loop functionality"""
    
    def test_display_recording_status(self):
        """Test recording status display"""

class TestPlaybackEngine:
    @pytest.mark.asyncio
    async def test_playback_sequence(self):
        """Test sequence playback"""
    
    @pytest.mark.asyncio
    async def test_precision_testing(self):
        """Test precision measurement during playback"""
```

#### `test_digitizer_data_structures.py`
```python
class TestDigitizedPoint:
    def test_creation(self):
        """Test DigitizedPoint creation"""
    
    def test_serialization(self):
        """Test JSON serialization/deserialization"""

class TestDigitizedSequence:
    def test_creation(self):
        """Test DigitizedSequence creation"""
    
    def test_validation(self):
        """Test sequence validation"""
    
    def test_serialization(self):
        """Test JSON persistence"""

class TestPlaybackStats:
    def test_creation(self):
        """Test PlaybackStats creation"""
    
    def test_assessment_calculation(self):
        """Test precision assessment scoring"""
```

#### `test_precision_analyzer.py`
```python
class TestPrecisionAnalyzer:
    def test_assess_precision(self):
        """Test precision assessment algorithms"""
    
    def test_calculate_path_complexity(self):
        """Test path complexity scoring"""
    
    def test_analyze_repeatability(self):
        """Test repeatability analysis"""
    
    def test_statistical_functions(self):
        """Test statistical analysis functions"""
```

#### `test_surface_mapping.py`
```python
class TestSurfaceMapping:
    def test_surface_point_creation(self):
        """Test SurfacePoint creation"""
    
    def test_surface_map_creation(self):
        """Test SurfaceMap creation"""
    
    def test_surface_statistics(self):
        """Test surface analysis functions"""

class TestEnhancedHeightMapGenerator:
    @pytest.mark.asyncio
    async def test_setup_standard_plotter(self):
        """Test plotter setup"""
    
    def test_calculate_z_from_pen_angle(self):
        """Test Z height calculation"""
    
    def test_calculate_bounds(self):
        """Test surface bounds calculation"""
```

### 3.3 Integration Tests Required

#### `test_digitizer_with_simulator.py`
```python
class TestDigitizerIntegration:
    @pytest.mark.asyncio
    async def test_full_recording_playback_cycle(self):
        """Test complete record/playback cycle with simulator"""
    
    @pytest.mark.asyncio
    async def test_precision_measurement(self):
        """Test precision measurement accuracy"""
    
    @pytest.mark.asyncio
    async def test_multi_axis_coordination(self):
        """Test multi-axis digitizing"""
    
    @pytest.mark.asyncio
    async def test_error_handling(self):
        """Test error handling during operations"""
```

### 3.4 Performance Tests

#### `test_digitizer_performance.py`
```python
class TestDigitizerPerformance:
    @pytest.mark.performance
    async def test_recording_sample_rates(self):
        """Test recording at various sample rates"""
    
    @pytest.mark.performance  
    async def test_playback_timing_accuracy(self):
        """Test playback timing precision"""
    
    @pytest.mark.performance
    async def test_large_sequence_handling(self):
        """Test performance with large datasets"""
```

## 4. Example File Modifications

### 4.1 Update Import Statements

**Before:**
```python
from motor_digitizer import MotorDigitizer, create_linear_axis
```

**After:**
```python
from mks_servo_can.digitizer import MotorDigitizer, create_linear_axis
# OR
from mks_servo_can import MotorDigitizer, create_linear_axis
```

### 4.2 Affected Example Files

#### `examples/height_map_generator_v2.py`
- Update imports to use library modules
- Remove local digitizer imports
- Update documentation strings

#### `examples/digitizer_precision_test.py` 
- Update imports to use library modules
- Simplify since core functionality is now in library
- Focus on application-specific test scenarios

### 4.3 New Example Files

#### `examples/basic_digitizer_example.py`
```python
"""
Basic MotorDigitizer Usage Example

Demonstrates core recording and playback functionality using the
library's built-in MotorDigitizer.
"""
import asyncio
from mks_servo_can import CANInterface, MotorDigitizer, create_linear_axis

async def main():
    can_if = CANInterface(use_simulator=True)
    await can_if.connect()
    
    digitizer = MotorDigitizer(can_if)
    
    # Add axes
    x_axis = create_linear_axis(can_if, 1, "X", 40.0)
    await digitizer.add_axis(x_axis)
    await digitizer.initialize_axes()
    
    # Record and playback
    await digitizer.start_recording(sample_rate=10.0)
    stats = await digitizer.playback_sequence(
        digitizer.current_sequence, 
        precision_test=True
    )
    
    print(f"Precision: {stats.average_position_error}")
    
    await digitizer.cleanup()
    await can_if.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
```

#### `examples/precision_testing_example.py`
```python
"""
Advanced Precision Testing Example

Shows how to use the PrecisionAnalyzer for detailed system characterization.
"""
# Implementation using library components
```

## 5. Documentation Updates

### 5.1 API Documentation

#### New Documentation Files

**`docs/user_guides/library/digitizer_overview.md`**
- Introduction to MotorDigitizer capabilities
- When to use digitizing vs. direct control
- Overview of recording/playback workflow

**`docs/user_guides/library/recording_playback.md`**
- Detailed recording procedures
- Playback configuration and options
- Data format specifications

**`docs/user_guides/library/precision_testing.md`**
- Precision testing methodologies
- Statistical analysis interpretation
- Performance optimization tips

**`docs/user_guides/library/surface_mapping.md`**
- Height mapping procedures
- Surface analysis capabilities
- Integration with existing workflows

#### Updated Documentation Files

**`docs/user_guides/library/multi_axis.md`**
- Add section on digitizer integration
- Multi-axis recording examples

**`docs/api_reference/library/digitizer.md`**
- Complete API reference for digitizer module
- Class hierarchies and relationships
- Method signatures and parameters

### 5.2 Tutorial Updates

**`docs/tutorials/precision_validation.md`**
- Step-by-step precision testing tutorial
- Interpreting results and troubleshooting
- Best practices for accurate measurements

**`docs/tutorials/manual_teaching.md`**
- Manual path teaching workflow
- Recording complex trajectories
- Playback optimization

### 5.3 README Updates

**Main Project README**
- Add MotorDigitizer to feature list
- Update installation instructions for optional dependencies
- Add precision testing to use cases

**Examples README**
- Update examples list with new digitizer examples
- Add cross-references to library documentation

## 6. Implementation Steps

### 6.1 Phase 1: Core Library Integration

1. **Create digitizer module structure**
   - Create `mks_servo_can/digitizer/` directory
   - Split current `motor_digitizer.py` into modular files
   - Implement proper imports and exports

2. **Update main library interface**
   - Modify `mks_servo_can/__init__.py`
   - Update version number
   - Add optional dependencies to setup.py

3. **Write unit tests**
   - Implement core unit tests for all components
   - Ensure 100% test coverage for public APIs
   - Add mocking for hardware-dependent functionality

### 6.2 Phase 2: Integration and Documentation

4. **Write integration tests**
   - End-to-end testing with simulator
   - Performance benchmarking
   - Error condition testing

5. **Update documentation**
   - Create new documentation files
   - Update existing documentation with digitizer references
   - Add tutorial content

6. **Update examples**
   - Modify existing examples to use library imports
   - Create new demonstration examples
   - Add command-line interfaces

### 6.3 Phase 3: Validation and Release

7. **Comprehensive testing**
   - Run full test suite including new digitizer tests
   - Performance validation
   - Documentation review

8. **Release preparation**
   - Update changelog
   - Tag release version
   - Update documentation deployment

## 7. Migration Compatibility

### 7.1 Backward Compatibility

To maintain compatibility during migration:

```python
# examples/motor_digitizer.py (deprecated wrapper)
"""
DEPRECATED: This module has moved to mks_servo_can.digitizer

This file provides backward compatibility. Please update your imports to:
    from mks_servo_can.digitizer import MotorDigitizer
"""
import warnings
from mks_servo_can.digitizer import *

warnings.warn(
    "motor_digitizer module is deprecated. "
    "Use 'from mks_servo_can.digitizer import MotorDigitizer' instead.",
    DeprecationWarning,
    stacklevel=2
)
```

### 7.2 Migration Script

```python
# migration_helper.py
"""
Helper script to update import statements in user code
"""
import os
import re
from pathlib import Path

def update_imports_in_file(filepath):
    """Update digitizer imports in a Python file"""
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Replace import patterns
    patterns = [
        (r'from motor_digitizer import', 'from mks_servo_can.digitizer import'),
        (r'import motor_digitizer', 'import mks_servo_can.digitizer as motor_digitizer'),
    ]
    
    for old_pattern, new_pattern in patterns:
        content = re.sub(old_pattern, new_pattern, content)
    
    with open(filepath, 'w') as f:
        f.write(content)

def migrate_project(project_path):
    """Migrate all Python files in a project"""
    for py_file in Path(project_path).rglob("*.py"):
        update_imports_in_file(py_file)
        print(f"Updated: {py_file}")
```

## 8. Quality Assurance

### 8.1 Code Quality Requirements

- **Test Coverage**: Minimum 95% for digitizer module
- **Documentation Coverage**: 100% for public APIs
- **Type Hints**: Complete type annotations for all functions
- **Linting**: Pass all flake8 and pylint checks
- **Performance**: No regression in existing functionality

### 8.2 Review Process

1. **Code Review**: Comprehensive review of all new modules
2. **Documentation Review**: Technical writing review of all documentation
3. **Testing Review**: Validation of test coverage and scenarios
4. **Performance Review**: Benchmarking against current examples

## 9. Risk Mitigation

### 9.1 Potential Risks

1. **Breaking Changes**: Risk of breaking existing example code
   - **Mitigation**: Maintain backward compatibility wrappers

2. **Performance Regression**: Risk of slower imports or runtime
   - **Mitigation**: Lazy imports and performance testing

3. **Test Complexity**: Risk of complex test dependencies
   - **Mitigation**: Mock external dependencies, isolated test environments

4. **Documentation Debt**: Risk of incomplete documentation
   - **Mitigation**: Documentation-driven development, review process

### 9.2 Rollback Plan

If issues arise during migration:
1. Revert library changes via git
2. Restore original examples functionality
3. Address identified issues in feature branch
4. Re-attempt migration with fixes

## 10. Success Criteria

### 10.1 Technical Criteria

- [ ] All existing tests pass
- [ ] New digitizer tests achieve >95% coverage
- [ ] No performance regression in core library
- [ ] All examples work with new imports
- [ ] Documentation builds without errors

### 10.2 User Experience Criteria

- [ ] Digitizer functionality accessible via `from mks_servo_can import MotorDigitizer`
- [ ] Clear migration path for existing users
- [ ] Comprehensive documentation and examples
- [ ] Backward compatibility maintained for 6 months minimum

### 10.3 Project Criteria

- [ ] Version 0.2.0 release successfully deployed
- [ ] All project stakeholders approve the changes
- [ ] No outstanding technical debt introduced
- [ ] Clear roadmap for future digitizer enhancements

This migration plan provides a comprehensive roadmap for moving the MotorDigitizer from examples into the core library while maintaining quality, compatibility, and user experience standards.