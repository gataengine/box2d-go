package box2d

import "errors"

func CreateWorld(def WorldDef) (*World, error) {
	return NewWorld(def)
}

func DestroyWorld(world *World) error {
	if world == nil {
		return ErrWorldInvalid
	}
	return world.Destroy()
}

func (id ContactID) GetData(world *World) (ContactData, error) {
	if world == nil {
		return ContactData{}, ErrWorldInvalid
	}
	world.mu.RLock()
	defer world.mu.RUnlock()
	if !IsWorldIDValid(world.id) {
		return ContactData{}, ErrWorldInvalid
	}
	for i, entry := range world.activeContactData {
		if entry.contactID.Index1 == id.Index1 && entry.contactID.Generation == id.Generation && entry.contactID.World0 == id.World0 {
			return world.contactDataFromPairLocked(world.activeContactPairs[i]), nil
		}
	}
	return ContactData{}, errors.New("box2d: contact id is invalid")
}
