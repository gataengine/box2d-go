package box2d_test

import (
	"testing"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

func TestIDs(t *testing.T) {
	RegisterFailHandler(Fail)
	RunSpecs(t, "Box2D ID Suite")
}
