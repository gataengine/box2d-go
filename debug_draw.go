package box2d

type DebugDraw struct {
	DrawShapes           bool
	DrawJoints           bool
	DrawAABBs            bool
	DrawMass             bool
	DrawContacts         bool
	DrawGraphColors      bool
	DrawContactNormals   bool
	DrawContactImpulses  bool
	DrawFrictionImpulses bool
}

func DefaultDebugDraw() DebugDraw {
	return DebugDraw{
		DrawShapes:           true,
		DrawJoints:           true,
		DrawAABBs:            false,
		DrawMass:             false,
		DrawContacts:         false,
		DrawGraphColors:      false,
		DrawContactNormals:   false,
		DrawContactImpulses:  false,
		DrawFrictionImpulses: false,
	}
}
