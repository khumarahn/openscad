// This file is a part of openscad. Everything implied is implied.
// Author: Alexey Korepanov <kaikaikai@yandex.ru>

#pragma once

#include <string>

#include "node.h"
#include "value.h"

class RoofNode : public AbstractPolyNode
{
public:
	VISITABLE();
	RoofNode(const ModuleInstantiation *mi, const std::shared_ptr<EvalContext> &ctx) : AbstractPolyNode(mi, ctx) {}
	std::string toString() const override;
	std::string name() const override { return "roof"; }

	double fa, fs, fn;
	std::string method;

	class roof_exception: public std::exception {
		public:
			roof_exception(const std::string message) : m (message) {};
			std::string message() {return m;}
		private:
			std::string m;
	};
};
