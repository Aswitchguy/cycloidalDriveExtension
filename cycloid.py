from tkinter import N
import adsk.core, adsk.fusion, adsk.cam, traceback, math


housingRollerSpacing = adsk.core.ValueCommandInput.cast(None)
housingRollerRadius = adsk.core.ValueCommandInput.cast(None)
housingRollerNumber = adsk.core.ValueCommandInput.cast(None)

outputRollerRadius = adsk.core.ValueCommandInput.cast(None)
outputRollerNumber = adsk.core.ValueCommandInput.cast(None)
outputRollerSpacing = adsk.core.ValueCommandInput.cast(None)

camRadius = adsk.core.ValueCommandInput.cast(None)
camTolerance = adsk.core.ValueCommandInput.cast(None)
camShaftRadius = adsk.core.ValueCommandInput.cast(None)

layerHeight = adsk.core.ValueCommandInput.cast(None)
layerNumber = adsk.core.ValueCommandInput.cast(None)
eccentricity = adsk.core.ValueCommandInput.cast(None)

_app = None
_ui  = None

# global set of event handlers to keep them referenced for the duration of the command
_handlers = []

class MyCommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input
        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# event handler that reacts to when the command is destroyed. This terminates the script.            
class MyCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            adsk.terminate()
        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class MyCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            cmd = adsk.core.Command.cast(args.command)
            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)        
            onInputChanged = MyCommandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)    
            inputs = cmd.commandInputs
            global housingRollerSpacing, housingRollerRadius, housingRollerNumber, outputRollerHoleRadius, outputRollerNumber, outputRollerSpacing, camRadius, camTolerance, camShaftRadius, layerHeight, layerConfiguration, layerTolerance, eccentricity

            # add inputs
            #inputs.addImageCommandInput('image', '', "resources/diagram.png")
            #message = '<div align="center">View the documentation <a href="https://github.com/olearyf/cycloidal-gears">here.</a></div>'
            #inputs.addTextBoxCommandInput('fullWidth_textBox', '', message, 1, True)            
            
            housingRollerSpacing = inputs.addValueInput('housingRollerSpacing', 'Housing Roller Spacing', 'mm', adsk.core.ValueInput.createByReal(6))
            housingRollerRadius = inputs.addValueInput('housingRollerRadius', 'Housing Roller Radius', 'mm', adsk.core.ValueInput.createByReal(0.5))
            housingRollerNumber = inputs.addValueInput('housingRollerNumber', 'Housing Roller Number', '', adsk.core.ValueInput.createByReal(20))

            outputRollerHoleRadius = inputs.addValueInput('outputRollerHoleRadius', 'Output Roller Hole Radius', 'mm', adsk.core.ValueInput.createByReal(0.7))
            outputRollerNumber = inputs.addValueInput('outputRollerNumber', 'Output Roller Number', '', adsk.core.ValueInput.createByReal(10))
            outputRollerSpacing = inputs.addValueInput('outputRollerSpacing', 'Output Roller Spacing', 'mm', adsk.core.ValueInput.createByReal(3.5))

            camRadius = inputs.addValueInput('camRadius', 'Cam Radius', 'mm', adsk.core.ValueInput.createByReal(1.25))
            camTolerance = inputs.addValueInput('camTolerance', 'Cam Tolerance', 'mm', adsk.core.ValueInput.createByReal(0))
            camShaftRadius = inputs.addValueInput('camShaftRadius', 'Cam Shaft Radius', 'mm', adsk.core.ValueInput.createByReal(0.25))

            layerHeight = inputs.addValueInput('layerHeight', 'Layer Height', 'mm', adsk.core.ValueInput.createByReal(1))
            layerConfiguration = inputs.addValueInput('layerConfiguration', 'Layer Configuration', '', adsk.core.ValueInput.createByReal(1))
            layerTolerance = inputs.addValueInput('layerTolerance', 'Layer Tolerance', '', adsk.core.ValueInput.createByReal(0))
            
            eccentricity = inputs.addValueInput('eccentricity', 'Eccentricity', 'mm', adsk.core.ValueInput.createByReal(0.2))

            onExecute = CycloidalGearCommandExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute) 
        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class CycloidalGearCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            global housingRollerSpacing, housingRollerRadius, housingRollerNumber, outputRollerHoleRadius, outputRollerNumber, outputRollerSpacing, camRadius, camTolerance, camShaftRadius, layerHeight, layerConfiguration, layerTolerance, eccentricity
            eventArgs = adsk.core.CommandEventArgs.cast(args)
            
            housingRollerSpacing = housingRollerSpacing.value
            housingRollerRadius = housingRollerRadius.value
            housingRollerNumber = int(housingRollerNumber.value)

            outputRollerHoleRadius = outputRollerHoleRadius.value
            outputRollerNumber = int(outputRollerNumber.value)
            outputRollerSpacing = outputRollerSpacing.value

            camRadius = camRadius.value
            camTolerance = camTolerance.value
            camShaftRadius = camShaftRadius.value

            layerHeight = layerHeight.value
            layerConfiguration = int(layerConfiguration.value)
            layerTolerance = layerTolerance.value

            eccentricity = eccentricity.value

            generate()
        except:
            if _ui:
                _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def getOffset(E, a=0):
    x = E * math.cos(a / (180 * math.pi))
    y = E * math.sin(a / (180 * math.pi))

    return (x, y)

def getPoint(t, R, Rr, E, N, a=0):
    #psi = -math.atan(math.sin((1 - N) * theta) / ((R / (E * N)) - math.cos((1 - N) * theta)))
    #x = R * math.cos(theta) - Rr * math.cos(theta - psi) - E * math.cos(N * theta)
    #y =  - R * math.sin(theta) + Rr * math.sin(theta - psi) + E * math.cos(N * theta)
    psi = math.atan2(math.sin((1-N)*t), ((R/(E*N))-math.cos((1-N)*t)))

    x = (R*math.cos(t))-(Rr*math.cos(t+psi))-(E*math.cos(N*t))
    y = (-R*math.sin(t))+(Rr*math.sin(t+psi))+(E*math.sin(N*t))

    x += getOffset(E, a)[0]
    y += getOffset(E, a)[1]
    
    #x = (R * math.cos(t)) - (r * math.cos(t + p)) - (E * math.cos(n * t))
    #y = (-R * math.sin(t)) + (r * math.sin(t + p)) + (E * math.sin(n * t))

    #x = (10*math.cos(t))-(1.5*math.cos(t+math.atan(math.sin(-9*t)/((4/3)-math.cos(-9*t)))))-(0.75*math.cos(10*t))
    #y = (-10*math.sin(t))+(1.5*math.sin(t+math.atan(math.sin(-9*t)/((4/3)-math.cos(-9*t)))))+(0.75*math.sin(10*t))
    return (x,y)

def generate():

    global housingRollerSpacing, housingRollerRadius, housingRollerNumber, outputRollerHoleRadius, outputRollerNumber, outputRollerSpacing, camRadius, camTolerance, camShaftRadius, layerHeight, layerConfiguration, layerTolerance, eccentricity

    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    # Get the root component of the active design.
    rootComp = design.rootComponent
    # Create a new sketch on the xy plane.
    sketches = rootComp.sketches
    xzPlane = rootComp.xZConstructionPlane
    sketch = sketches.add(xzPlane)
    points = adsk.core.ObjectCollection.create()
    # Enter variables here. E.g. E = 50
    startRange = 0  # Start of range to be evaluated.
    endRange = 2 * math.pi  # End of range to be evaluated.
    splinePoints = 200  # Number of points that splines are generated.
    # WARNING: Using more than a few hundred points may cause your system to hang.

    rotorOcc = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rotor = rotorOcc.component
    rotor.name = "rotor"
    rotorSketch = rotor.sketches.add(xzPlane)
    rotorSketch.isComputeDeferred = True

    eccentricSpacerOcc = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    eccentricSpacer = eccentricSpacerOcc.component
    eccentricSpacer.name = "eccentricSpacer"
    eccentricSpacerSketch = eccentricSpacer.sketches.add(xzPlane)
    eccentricSpacerSketch.isComputeDeferred = True

    outputRollersOcc = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    outputRollers = outputRollersOcc.component
    outputRollers.name = "outputRollers"
    outputRollersSketch = outputRollers.sketches.add(xzPlane)
    outputRollersSketch.isComputeDeferred = True

    housingRollersOcc = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    housingRollers = housingRollersOcc.component
    housingRollers.name = "housingRollers"
    housingRollersSketch = housingRollers.sketches.add(xzPlane)
    housingRollersSketch.isComputeDeferred = True

    for i in range(splinePoints):
        t = i / splinePoints * endRange
        x, y = getPoint(t, housingRollerSpacing, housingRollerRadius, eccentricity, housingRollerNumber)
        points.add(adsk.core.Point3D.create(x, y, 0))
    # Generates the spline curve
    # Create sketch lines based on the points
    #for i in range(len(points) - 1):
        #sketch.sketchCurves.sketchLines.addByTwoPoints(points[i], points[i + 1])
    
    #sketch.sketchCurves.sketchLines.addByTwoPoints(points[len(points)-1], points[0])
    spline = rotorSketch.sketchCurves.sketchFittedSplines.add(points)
    spline.isClosed = True
    
    center = getOffset(eccentricity, 0)
    rotorCircles = rotorSketch.sketchCurves.sketchCircles
    circle1 = rotorCircles.addByCenterRadius(adsk.core.Point3D.create(center[0], center[1], 0), camRadius)
    outputRollersCircles = outputRollersSketch.sketchCurves.sketchCircles
    housingRollersCircles = housingRollersSketch.sketchCurves.sketchCircles

    for i in range(housingRollerNumber):
        h = housingRollerSpacing * math.cos((2 * math.pi * i) / housingRollerNumber)
        k = housingRollerSpacing * math.sin((2 * math.pi * i) / housingRollerNumber)
        housingRollersCircles.addByCenterRadius(adsk.core.Point3D.create(h, k, 0), housingRollerRadius)
    
    for i in range(outputRollerNumber):
        xo = getOffset(eccentricity)[0]
        yo = getOffset(eccentricity)[1]
        
        h = (outputRollerSpacing * math.cos((i * 2 * math.pi) / outputRollerNumber))
        k = (outputRollerSpacing * math.sin((i * 2 * math.pi) / outputRollerNumber))
        rotorCircles.addByCenterRadius(adsk.core.Point3D.create(h + xo, k + yo, 0), outputRollerHoleRadius)
        outputRollersCircles.addByCenterRadius(adsk.core.Point3D.create(h, k, 0), outputRollerHoleRadius - eccentricity)

    eccentricSpacerCircles = eccentricSpacerSketch.sketchCurves.sketchCircles
    circle1 = eccentricSpacerCircles.addByCenterRadius(adsk.core.Point3D.create(center[0], center[1], 0), camRadius)
    circle2 = eccentricSpacerCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), camShaftRadius)

    # Stop the sketch command and rebuild
    rotorSketch.isComputeDeferred = False
    eccentricSpacerSketch.isComputeDeferred = False
    outputRollersSketch.isComputeDeferred = False
    housingRollersSketch.isComputeDeferred = False

    rotorSketchProfile = rotorSketch.profiles.item(0)
    extrudes = rotor.features.extrudeFeatures
    extrude1 = extrudes.addSimple(rotorSketchProfile, adsk.core.ValueInput.createByReal(layerHeight), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    
    eccentricSpacerSketchProfile = eccentricSpacerSketch.profiles.item(0)
    extrudes = eccentricSpacer.features.extrudeFeatures
    extrude1 = extrudes.addSimple(eccentricSpacerSketchProfile, adsk.core.ValueInput.createByReal(layerHeight), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    
    extrudes = outputRollers.features.extrudeFeatures
    for i in range(outputRollersSketch.profiles.count):
        outputRollerSketchProfile = outputRollersSketch.profiles.item(i)
        extrude1 = extrudes.addSimple(outputRollerSketchProfile, adsk.core.ValueInput.createByReal(layerHeight), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    extrudes = housingRollers.features.extrudeFeatures
    for i in range(housingRollersSketch.profiles.count):
        housingRollersSketchProfile = housingRollersSketch.profiles.item(i)
        extrude1 = extrudes.addSimple(housingRollersSketchProfile, adsk.core.ValueInput.createByReal(layerHeight), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

def run(context):
    try:
        global _app, _ui
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        cmdDef = _ui.commandDefinitions.itemById('cmdInputsCyclGear')
        if not cmdDef:
            cmdDef = _ui.commandDefinitions.addButtonDefinition('cmdInputsCyclGear', 'Cycloidal Gear', 'Creates a cycloidal gear based on input parameters.')
        onCommandCreated = MyCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)
        cmdDef.execute()
        adsk.autoTerminate(False)
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Run the script
#run(app)
