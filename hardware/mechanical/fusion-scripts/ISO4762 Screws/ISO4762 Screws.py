"""ISO 4762 socket head cap screws: builds each size in a new Fusion design and saves it to the active project.
Run from Fusion: Utilities > Scripts and Add-Ins > + > Script or add-in from my computer > this folder > Run.
Head sits on the XY plane (head up to +k, shank down to -L), modelled 6g thread over the full shank."""
import adsk.core, adsk.fusion, math

# name: (thread d, pitch, length L, head dia dk, head height k, hex key s, socket depth t, tip chamfer) in mm
SCREWS = {
    'M3x6':   (3.0, 0.5,  6.0, 5.5, 3.0, 2.5, 1.3, 0.3),
    'M2.5x6': (2.5, 0.45, 6.0, 4.5, 2.5, 2.0, 1.1, 0.25),
}

def build(app, name, d, p, L, dk, k, s, t, cham):
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    P = adsk.core.Point3D.create
    cm = lambda mm: adsk.core.ValueInput.createByReal(mm / 10)
    ext = root.features.extrudeFeatures
    NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                      adsk.fusion.FeatureOperations.JoinFeatureOperation,
                      adsk.fusion.FeatureOperations.CutFeatureOperation)
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), d / 20)
    body = ext.addSimple(sk.profiles.item(0), cm(-L), NEW).bodies.item(0)
    body.name = name + ' ISO4762'
    sk2 = root.sketches.add(root.xYConstructionPlane)
    sk2.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), dk / 20)
    profs = adsk.core.ObjectCollection.createWithArray([sk2.profiles.item(i) for i in range(sk2.profiles.count)])
    ext.addSimple(profs, cm(k), JOIN)
    pl = root.constructionPlanes.createInput(); pl.setByOffset(root.xYConstructionPlane, cm(k))
    sk3 = root.sketches.add(root.constructionPlanes.add(pl))
    R = s / 20 / math.cos(math.radians(30))                       # hex circumradius from across-flats s
    pts = [P(R * math.cos(math.radians(60 * i)), R * math.sin(math.radians(60 * i)), 0) for i in range(6)]
    for i in range(6):
        sk3.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % 6])
    ext.addSimple(sk3.profiles.item(0), cm(-t), CUT)
    body = root.bRepBodies.item(0)
    tip = [e for e in body.edges if isinstance(e.geometry, adsk.core.Circle3D) and abs(e.geometry.center.z + L / 10) < 1e-6]
    ch = root.features.chamferFeatures.createInput2()
    ch.chamferEdgeSets.addEqualDistanceChamferEdgeSet(adsk.core.ObjectCollection.createWithArray(tip), cm(cham), False)
    root.features.chamferFeatures.add(ch)
    face = [f for f in body.faces if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius - d / 20) < 1e-6][0]
    tf = root.features.threadFeatures
    designation = 'M%gx%g' % (d, p)
    ti = tf.createInput(face, tf.createThreadInfo(False, 'ISO Metric profile', designation, '6g'))
    ti.isModeled = True; ti.isFullLength = True
    tf.add(ti)
    bad = [des.timeline.item(i).name for i in range(des.timeline.count)
           if des.timeline.item(i).healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState]
    if bad:
        raise RuntimeError('%s: unhealthy features %s' % (name, bad))
    app.activeViewport.fit()
    doc.saveAs(name, app.data.activeProject.rootFolder, 'ISO 4762 socket head cap screw %s-6g x %g mm' % (designation, L), '')
    return designation

def run(context):
    app = adsk.core.Application.get()
    done = [name + ' (' + build(app, name, *dims) + ')' for name, dims in SCREWS.items()]
    app.userInterface.messageBox('Built and saved to project "%s":\n%s' % (app.data.activeProject.name, '\n'.join(done)))
